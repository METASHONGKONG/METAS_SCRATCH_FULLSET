using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;



using System.Management;
using System.Threading;
using System.Text.RegularExpressions;
using System.Net;
using System.Net.Sockets;
using System.Net.NetworkInformation;
using System.IO.Ports;



namespace MakkoLocalServer
{

    class SerialDeviceEnumeration
    {
        static public void Start()
        {
            deviceList = new List<String>();

            // First Scan
            foreach (String port in SerialPort.GetPortNames()) AddDevice(port);

            // Observe Changes
            deviceChangeEventWatcher = new ManagementEventWatcher();
            deviceChangeEventWatcher.EventArrived += new EventArrivedEventHandler(deviceChangeEventArrived);
            deviceChangeEventWatcher.Query = new WqlEventQuery("__InstanceOperationEvent", new TimeSpan(0, 0, 3), @"TargetInstance ISA 'Win32_USBControllerDevice'");
            deviceChangeEventWatcher.Start();
        }
        static public void Stop()
        {
            deviceChangeEventWatcher.Stop();
            deviceChangeEventWatcher.Dispose();
        }

        static private List<String> deviceList;
        static private ManagementEventWatcher deviceChangeEventWatcher;
        static private void deviceChangeEventArrived(object sender, EventArrivedEventArgs e)
        {
            try {
                foreach (PropertyData pdData in e.NewEvent.Properties) {
                    if (pdData.Name != "TargetInstance") continue;

                    List<String> newList = SerialPort.GetPortNames().ToList<String>();
                    List<String> deviceAdded = newList.Except(deviceList).ToList();
                    List<String> deviceRemoved = deviceList.Except(newList).ToList();

                    foreach (String port in deviceAdded) AddDevice(port);
                    foreach (String port in deviceRemoved) RemoveDevice(port);
                }
            } catch(Exception) { }
        }
        static private void AddDevice(String port)
        {
            deviceList.Add(port);
            ManagementObjectSearcher searcher = new ManagementObjectSearcher("SELECT * FROM Win32_SerialPort WHERE Description = 'Arduino Uno' AND DeviceID = '" + port + "'");
            if (searcher.Get().Count > 0)
                DeviceManager.AddDevice(port, typeof(ArduinoUnoSerialDevice)); //Confirmed Arduino Uno
            else
                DeviceManager.AddDevice(port, typeof(MetasArduinoSerialDevice)); //Default
        }
        static private void RemoveDevice(String port)
        {
            deviceList.Remove(port);
            DeviceManager.RemoveDevice(port);
        }
    }
    class TCPIPDeviceEnumeration
    {
        static private Int32 beaconPort = 1002;
        static private Int32 replyPort = 1003;
        static private IPAddress multicastAddress = IPAddress.Parse("239.10.1.101");

        static List<NetworkInterfaceSocket> interfaces = new List<NetworkInterfaceSocket>();

        static public void Start()
        {
            foreach (NetworkInterface netInterface in NetworkInterface.GetAllNetworkInterfaces())
            {
                //Skip Loopback (such as 127.0.0.1)
                if (netInterface.NetworkInterfaceType == NetworkInterfaceType.Loopback) continue;

                //Skip Disabled Interface
                if (netInterface.OperationalStatus != OperationalStatus.Up) continue;

                //Skip Not Supporting Multicast
                if (!netInterface.SupportsMulticast) continue;

                //Skip Not Supporting IPv4
                if (!netInterface.Supports(NetworkInterfaceComponent.IPv4)) continue;

                //Enumerate Addresses
                IPInterfaceProperties ipProps = netInterface.GetIPProperties();
                foreach (UnicastIPAddressInformation addr in ipProps.UnicastAddresses)
                {
                    //Skip Non-IPv4 Addresses
                    if (addr.Address.AddressFamily != System.Net.Sockets.AddressFamily.InterNetwork) continue;

                    //Create Socket
                    interfaces.Add(new NetworkInterfaceSocket(addr.Address));
                }
            }

            // Setup Scan Timer
            System.Timers.Timer timersTimer = new System.Timers.Timer();
            timersTimer.AutoReset = true;
            timersTimer.Interval = 5000;
            timersTimer.Elapsed += new System.Timers.ElapsedEventHandler(scan);
            timersTimer.Start();

            // Initial Scan
            scan(null, null);
        }
        static public void Stop()
        {
            interfaces.Clear();
        }

        static private void scan(object sender, System.Timers.ElapsedEventArgs e)
        {
            foreach (NetworkInterfaceSocket interfaceSocket in interfaces)
            {
                interfaceSocket.SendBeacon();
            }
        }

        private class NetworkInterfaceSocket
        {
            //Private Storage
            private IPAddress interfaceIP;
            private UdpClient udpclient = new UdpClient();

            //Constructor
            public NetworkInterfaceSocket(IPAddress InterfaceAddress)
            {
                interfaceIP = InterfaceAddress;
                udpclient.Client.Bind(new IPEndPoint(InterfaceAddress, replyPort));
                udpclient.JoinMulticastGroup(multicastAddress);
                udpclient.BeginReceive(new AsyncCallback(ReceiveCallback), null);
            }
            public void SendBeacon()
            {
                try {
                    Byte[] buffer = interfaceIP.GetAddressBytes();
                    udpclient.Send(buffer, buffer.Length, new IPEndPoint(multicastAddress, beaconPort));
                } catch(Exception) { }
            }
            private void ReceiveCallback(IAsyncResult ar)
            {
                try {
                    IPEndPoint e = new IPEndPoint(IPAddress.Any, 0);
                    Byte[] receiveBytes = udpclient.EndReceive(ar, ref e);
                    DeviceManager.AddDevice(new IPAddress(receiveBytes).ToString(), typeof(IOTBoxDevice));
                } catch (Exception) {
                } finally {
                    udpclient.BeginReceive(new AsyncCallback(ReceiveCallback), null);
                }
            }
        }
    }


}
