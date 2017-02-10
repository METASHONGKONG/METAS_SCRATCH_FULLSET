using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;



using System.Threading;
using System.Reflection;
using System.Net.NetworkInformation;
using System.Net;


namespace MakkoLocalServer
{
    class DeviceManager
    {
        static public void Start()
        {
            // Get network interfaces for metas x nodemcu
            foreach (NetworkInterface netInterface in NetworkInterface.GetAllNetworkInterfaces())
            {
                //Skip Loopback (such as 127.0.0.1)
                if (netInterface.NetworkInterfaceType == NetworkInterfaceType.Loopback) continue;

                //Skip Disabled Interface
                if (netInterface.OperationalStatus != OperationalStatus.Up) continue;

                //Skip Not Supporting IPv4
                if (!netInterface.Supports(NetworkInterfaceComponent.IPv4)) continue;

                //Enumerate Addresses
                IPInterfaceProperties ipProps = netInterface.GetIPProperties();
                foreach (UnicastIPAddressInformation addr in ipProps.UnicastAddresses)
                {
                    //Skip Non-IPv4 Addresses
                    if (addr.Address.AddressFamily != System.Net.Sockets.AddressFamily.InterNetwork) continue;

                    //Create Socket
                    if (interfaceIPAddress == null || addr.Address.GetAddressBytes()[0] > interfaceIPAddress.GetAddressBytes()[0]) interfaceIPAddress = addr.Address;
                }
            }
        }
        static public void Stop()
        {
            // End All Async Threads
            AbortAllAsync();

            // Disconnect All Devices
            lock (DevicesConnected)
            {
                foreach (Device _device in DevicesConnected)
                {
                    _device.Disconnect();
                }
                DevicesConnected.Clear();
            }
        }

        static public IPAddress interfaceIPAddress;

        // Thread Safe Add/Remove Methods & Device Provider
        static public void AddDevice(String _identifier, Type _deviceType)
        {
            if (!_deviceType.IsSubclassOf(typeof(Device))) return;
            _identifier = _identifier.ToUpper();

            lock (DevicesFound)
            {
                if (!DevicesFound.Exists(x => (x.Key == _identifier && x.Value == _deviceType)))
                {
                    DevicesFound.Add(new KeyValuePair<String, Type>(_identifier, _deviceType));
                    ReportDevicesFound();
                }
            }
        }
        static public void RemoveDevice(String _identifier)
        {
            _identifier = _identifier.ToUpper();
            lock (DevicesFound)
            {
                // Manage DevicesFound List
                Int32 numDeleted = DevicesFound.RemoveAll(x => x.Key == _identifier);

                // Update Flash
                if (numDeleted > 0) ReportDevicesFound();

                // Disconnect Device & Manage DevicesConnected List
                DisconnectDeviceByIdentifier(_identifier);
            }
        }
        static private void ReportDevicesFound(Int32? packetindex = null)
        {
            if (packetindex == null) {
                DevicesUpdateCallbackPacket returnpck = new DevicesUpdateCallbackPacket();
                lock (DevicesFound) foreach (KeyValuePair<String, Type> _deviceInfo in DevicesFound) returnpck.AddDevice(_deviceInfo.Key, (String)_deviceInfo.Value.GetField("Description").GetValue(null));
                FlashManager.AddResponse(returnpck);
            } else {
                ScanDevicesResponsePacket returnpck = new ScanDevicesResponsePacket(packetindex ?? 0);
                lock (DevicesFound) foreach (KeyValuePair<String, Type> _deviceInfo in DevicesFound) returnpck.AddDevice(_deviceInfo.Key, (String)_deviceInfo.Value.GetField("Description").GetValue(null));
                FlashManager.AddResponse(returnpck);
            }
        }
        static volatile private List<KeyValuePair<String, Type>> DevicesFound = new List<KeyValuePair<String, Type>>();

        // Connected Devices Management
        static private Device CreateDeviceManually(String _identifier)
        {
            // Can check identifier format and create different devices

            // Metas X NodeMCU
            String standardID = _identifier.Replace(" ", "").Replace("-", "").ToUpper().Replace('O', '0').Replace('I', '1').Replace('L', '1').Replace('Q', '9');
            if (System.Text.RegularExpressions.Regex.IsMatch(standardID, @"\A\b[0-9a-fA-F]{6}\b\Z")) {
                if (interfaceIPAddress == null) throw new ClassifiedException(Error.System_SubNetNotFound);
                return new MetasNodeMCUDevice(_identifier);
            }

            // Metas X NodeMCU (IP Version)
            System.Net.IPAddress address;
            if (System.Net.IPAddress.TryParse(_identifier, out address)) return new MetasNodeMCUIPDevice(_identifier);

            // No Match (Invalid)
            return null;
        }
        static private Device GetDeviceByIdentifier(String _identifier)
        {
            lock (DevicesConnected) foreach (Device _device in DevicesConnected) if (_device.GetIdentifier() == _identifier.ToUpper()) return _device;
            throw new ClassifiedException(Error.Input_Identifier_NotFound);
        }
        static private Device GetDeviceByIdentifierWithCreation(String _identifier)
        {
            _identifier = _identifier.ToUpper();
            lock (DevicesConnected)
            lock (DevicesFound)
            {
                foreach (Device _device in DevicesConnected) if (_device.GetIdentifier() == _identifier.ToUpper()) return _device;
                
                KeyValuePair<String, Type> entryFound = DevicesFound.Find(x => x.Key == _identifier);
                if (!entryFound.Equals(default(KeyValuePair<String, Type>))) return (Device)Activator.CreateInstance(entryFound.Value, entryFound.Key);

                return CreateDeviceManually(_identifier);
            }
        }
        static private void AddDeviceToConnected(Device _device)
        {
            lock (DevicesConnected) DevicesConnected.Add(_device);
        }
        static private void DisconnectDeviceByIdentifier(String _identifier)
        {
            try
            {
                Device _foundDevice = GetDeviceByIdentifier(_identifier);
                if (_foundDevice != null)
                {
                    DevicesConnected.Remove(_foundDevice);
                    _foundDevice.Disconnect();
                }
            } catch(Exception) { }
        }
        static volatile private List<Device> DevicesConnected = new List<Device>();
        
        // Aysnc Job Management
        static private List<Thread> AsyncThreadPool = new List<Thread>();
        static private void CreateAsync(ThreadStart asyncJob)
        {
            Thread newthread = new Thread(asyncJob);
            AsyncThreadPool.Add(newthread);
            newthread.Start();
        }
        static private void AbortAllAsync()
        {
            foreach (Thread _thread in AsyncThreadPool) _thread.Abort();
            foreach (Thread _thread in AsyncThreadPool) _thread.Join();
            AsyncThreadPool.Clear();
        }

        // Packet Analyser
        static public void PacketAnalyser(RequestPacket newPacket)
        {
            try
            {
                #region Handle Packet Index
                if (newPacket.Index == null)
                {
                    FlashManager.AddResponse(new FailCallbackPacket(new ClassifiedException(Error.Input_NoPacketIndex)));
                    return;
                }
                Int32 packetindex = newPacket.Index ?? 0;
                #endregion

                // Handle Packet Types
                #region scan-devices
                if (newPacket.Tag == "scan-devices")
                {
                    ReportDevicesFound(packetindex);
                }
                #endregion
                #region connect-device
                else if (newPacket.Tag == "connect-device")
                {
                    String packetDeviceID = newPacket.Args[0].Value;
                    Int32 cachedpacketindex = packetindex;

                    CreateAsync(delegate() {
                        #region Abortable Thread Operation
                        Exception e = null;
                        try
                        {
                            Device targetDevice = GetDeviceByIdentifierWithCreation(packetDeviceID);
                            if (targetDevice == null) throw new ClassifiedException(Error.Input_Identifier_Invalid);
                            AddDeviceToConnected(targetDevice);
                            targetDevice.Connect();
                        }
                        catch (Exception _e) { e = _e; }
                        finally {
                            if (e == null)
                                FlashManager.AddResponse(new GenericSuccessResponsePacket(cachedpacketindex));
                            else
                                FlashManager.AddResponse(new GenericFailResponsePacket(cachedpacketindex, e));
                        }
                        #endregion
                    });
                }
                #endregion
                #region disconnect-device
                else if (newPacket.Tag == "disconnect-device")
                {
                    DisconnectDeviceByIdentifier(newPacket.Args[0].Value);
                    FlashManager.AddResponse(new GenericSuccessResponsePacket(packetindex));
                }
                #endregion
                #region reset-device
                else if (newPacket.Tag == "reset-device")
                {
                    try
                    {
                        AbortAllAsync();
                        Device targetDevice = GetDeviceByIdentifier(newPacket.Args[0].Value);
                        targetDevice.ResetPinState();
                    }
                    catch (Exception e)
                    {
                        FlashManager.AddResponse(new GenericFailResponsePacket(packetindex, e));
                        return;
                    }
                    FlashManager.AddResponse(new GenericSuccessResponsePacket(packetindex));
                }
                #endregion
                #region set-digital-pin
                else if (newPacket.Tag == "set-digital-pin")
                {
                    try
                    {
                        Device targetDevice = GetDeviceByIdentifier(newPacket.Args[0].Value);
                        targetDevice.SetDigitalPin(Int32.Parse(newPacket.Args[1].Value), LogicHelper.Parse(newPacket.Args[2].Value));
                    }
                    catch (Exception e)
                    {
                        FlashManager.AddResponse(new GenericFailResponsePacket(packetindex, e));
                        return;
                    }
                    FlashManager.AddResponse(new GenericSuccessResponsePacket(packetindex));
                }
                #endregion
                #region set-pwm-pin
                else if (newPacket.Tag == "set-pwm-pin")
                {
                    Int32 value = 0;
                    try { value = (Int32) Math.Round(Double.Parse(newPacket.Args[2].Value)); }
                    catch (Exception) {
                        FlashManager.AddResponse(new GenericFailResponsePacket(packetindex, new ClassifiedException(Error.Input_NotNumeric)));
                        return;
                    }

                    try
                    {
                        Device targetDevice = GetDeviceByIdentifier(newPacket.Args[0].Value);
                        targetDevice.SetPWMPin(Int32.Parse(newPacket.Args[1].Value), value);
                    }
                    catch (Exception e)
                    {
                        FlashManager.AddResponse(new GenericFailResponsePacket(packetindex, e));
                        return;
                    }
                    FlashManager.AddResponse(new GenericSuccessResponsePacket(packetindex));
                }
                #endregion
                #region read-digital-pin
                else if (newPacket.Tag == "read-digital-pin")
                {
                    Int32 cachedpacketindex = packetindex;
                    String packetDeviceID = newPacket.Args[0].Value;
                    Int32 pin = Int32.Parse(newPacket.Args[1].Value);
                    Logic result = Logic.LOW;

                    CreateAsync(delegate() {
                        #region Abortable Thread Operation
                        Exception e = null;
                        try
                        {
                            Device targetDevice = GetDeviceByIdentifier(packetDeviceID);
                            result = targetDevice.ReadDigitalPin(pin);
                        }
                        catch (Exception _e) { e = _e; }
                        finally
                        {
                            if (e == null)
                            {
                                FlashManager.AddResponse(new DigitalReadSuccessResponsePacket(cachedpacketindex, result));
                            }
                            else
                            {
                                FlashManager.AddResponse(new GenericFailResponsePacket(cachedpacketindex, e));
                            }
                        }
                        #endregion
                    });
                }
                #endregion
                #region read-analog-pin
                else if (newPacket.Tag == "read-analog-pin")
                {
                    Int32 cachedpacketindex = packetindex;
                    String packetDeviceID = newPacket.Args[0].Value;
                    Int32 pin = Int32.Parse(newPacket.Args[1].Value);
                    Int32 result = 0;

                    CreateAsync(delegate() {
                        #region Abortable Thread Operation
                        Exception e = null;
                        try
                        {
                            Device targetDevice = GetDeviceByIdentifier(packetDeviceID);
                            result = targetDevice.ReadAnalogPin(pin);
                        }
                        catch (Exception _e) { e = _e; }
                        finally
                        {
                            if (e == null)
                            {
                                FlashManager.AddResponse(new AnalogReadSuccessResponsePacket(cachedpacketindex, result));
                            }
                            else
                            {
                                FlashManager.AddResponse(new GenericFailResponsePacket(cachedpacketindex, e));
                            }
                        }
                        #endregion
                    });
                }
                #endregion
                #region read-i2c
                else if (newPacket.Tag == "read-i2c")
                {
                    Int32 cachedpacketindex = packetindex;
                    String packetDeviceID = newPacket.Args[0].Value;
                    Byte i2c_address = (Byte)Convert.ToInt32(newPacket.Args[1].Value, 16);
                    Byte i2c_register = (Byte)Convert.ToInt32(newPacket.Args[2].Value, 16);
                    Int32 i2c_numofbytes = Int32.Parse(newPacket.Args[3].Value);
                    List<Byte> result = new List<Byte>();

                    CreateAsync(delegate()
                    {
                        #region Abortable Thread Operation
                        Exception e = null;
                        try
                        {
                            Device targetDevice = GetDeviceByIdentifier(packetDeviceID);
                            result = targetDevice.I2CRead(i2c_address, i2c_register, i2c_numofbytes);
                        }
                        catch (Exception _e) { e = _e; }
                        finally
                        {
                            if (e == null)
                            {
                                FlashManager.AddResponse(new I2CReadSuccessResponsePacket(cachedpacketindex, result));
                            }
                            else
                            {
                                FlashManager.AddResponse(new GenericFailResponsePacket(cachedpacketindex, e));
                            }
                        }
                        #endregion
                    });
                }
                #endregion
                #region write-i2c
                else if (newPacket.Tag == "write-i2c")
                {
                    Byte i2c_address = (Byte)Convert.ToInt32(newPacket.Args[1].Value);
                    Byte i2c_register = (Byte)Convert.ToInt32(newPacket.Args[2].Value);
                    List<Byte> i2c_data = new List<Byte>();
                    foreach (String _data in newPacket.Args[3].Value.Split(' ')) i2c_data.Add((Byte)Convert.ToInt32(_data, 16));

                    try
                    {
                        Device targetDevice = GetDeviceByIdentifier(newPacket.Args[0].Value);
                        targetDevice.I2CWrite(i2c_address, i2c_register, i2c_data);
                    }
                    catch (Exception e)
                    {
                        FlashManager.AddResponse(new GenericFailResponsePacket(packetindex, e));
                        return;
                    }
                    FlashManager.AddResponse(new GenericSuccessResponsePacket(packetindex));
                }
                #endregion
                #region move-servo
                else if (newPacket.Tag == "move-servo")
                {
                    Double value = 0;

                    // TODO: Put into the try for common error handling
                    if(!Double.TryParse(newPacket.Args[2].Value, out value))
                        FlashManager.AddResponse(new GenericFailResponsePacket(packetindex, new ClassifiedException(Error.Input_NotNumeric)));

                    try
                    {
                        Device targetDevice = GetDeviceByIdentifier(newPacket.Args[0].Value);
                        targetDevice.SetServoPin(Int32.Parse(newPacket.Args[1].Value), (Int32)Math.Round(value));
                    }
                    catch (Exception e)
                    {
                        FlashManager.AddResponse(new GenericFailResponsePacket(packetindex, e));
                        return;
                    }
                    FlashManager.AddResponse(new GenericSuccessResponsePacket(packetindex));
                }
                #endregion
                #region device-function
                else if (newPacket.Tag == "device-function")
                {
                    Int32 returnvalue = 0;
                    try
                    {
                        Device targetDevice = GetDeviceByIdentifier(newPacket.Args[0].Value);
                        returnvalue = targetDevice.CallFunction(newPacket.Args[1].Value);
                    }
                    catch (Exception e)
                    {
                        FlashManager.AddResponse(new GenericFailResponsePacket(packetindex, e));
                        return;
                    }
                    FlashManager.AddResponse(new FunctionSuccessResponsePacket(packetindex, returnvalue));
                }
                #endregion
                #region send-raw
                else if (newPacket.Tag == "send-raw")
                {
                    List<Byte> rawdata = new List<Byte>();
                    foreach (String _data in newPacket.Args[1].Value.Split(' ')) rawdata.Add((Byte)Convert.ToInt32(_data, 16));

                    try
                    {
                        Device targetDevice = GetDeviceByIdentifier(newPacket.Args[0].Value);
                        targetDevice.SendRaw(rawdata);
                    }
                    catch (Exception e)
                    {
                        FlashManager.AddResponse(new GenericFailResponsePacket(packetindex, e));
                        return;
                    }
                    FlashManager.AddResponse(new GenericSuccessResponsePacket(packetindex));
                }
                #endregion
                #region Uknown Packet
                else
                {
                    FlashManager.AddResponse(new GenericFailResponsePacket(packetindex, new ClassifiedException(Error.Input_UnkownPacket)));
                }
                #endregion

            } catch(Exception) { }
        }

        // Thread Safe Callbacks
        static public void DeviceResetCallback(Device _device)
        {
            FlashManager.AddResponse(new FailCallbackPacket(new ClassifiedException(Error.Device_Restarted)));
        }
        static public void DeviceDigitalReadCallback(Device _device, Int32 _pin, Logic _value)
        {
            FlashManager.AddResponse(new DigitalReadCallbackPacket(_device.GetIdentifier(), _pin, _value));
        }

    }

    
}
