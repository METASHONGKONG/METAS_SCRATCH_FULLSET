using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;



using System.IO.Ports;
using System.Threading;
using System.Net;
using System.Net.Sockets;
using System.IO;


namespace MakkoLocalServer
{
    enum PinType { UNDEFINED, INPUT, OUTPUT, ANALOG, PWM, SERVO, I2C, ONEWIRE, STEPPER, ENCODER, MOTOR }
    enum Logic { LOW, HIGH }
    static class LogicHelper
    {
        static public Logic Parse(String _input)
        {
            return (_input == "1") ? Logic.HIGH : Logic.LOW;
        }
        static public Logic Parse(Boolean _input)
        {
            return _input ? Logic.HIGH : Logic.LOW;
        }
        static public String ToString(Logic _input)
        {
            return (_input == Logic.HIGH) ? "1" : "0";
        }
        static public Byte ToByte(Logic _input)
        {
            return (_input == Logic.HIGH) ? (Byte)1 : (Byte)0;
        }
    }

    abstract class Device : IEquatable<Device>
    {
        private String Identifier; // Identifier
        protected Device(String _identifier)
        {
            Identifier = _identifier.ToUpper();
        }
        public String GetIdentifier()
        {
            return Identifier;
        }
        public override Int32 GetHashCode()
        {
            return GetIdentifier().GetHashCode();
        }
        public override Boolean Equals(Object other)
        {
            if (other == null) return false;
            return (this.GetIdentifier() == ((Device)other).GetIdentifier());
        }
        public Boolean Equals(Device other)
        {
            if (other == null) return false;
            return (this.GetIdentifier() == other.GetIdentifier());
        }

        private Object ThreadSafeLocker = new Object();
        protected GenericConnection connection;
        protected GenericProtocol protocol;


        public virtual void Connect()           //Thread Safe
        {
            lock (ThreadSafeLocker)
            {
                connection.Connect();
                protocol.Connect();
            }
        }
        public virtual void Disconnect()        //Thread Safe
        {
            lock (ThreadSafeLocker)
            {
                protocol.Disconnect();
                connection.Disconnect();
            }
        }

        protected Dictionary<PinType, List<Int32>> PinResources = new Dictionary<PinType,List<Int32>>();
        protected List<String> FunctionResources = new List<String>();
        public virtual void ResetPinState()
        {
            lock (ThreadSafeLocker) protocol.ResetPinState();
        }                                           //Thread Safe
        public virtual Int32 ReadAnalogPin(Int32 pin)                                    //Thread Safe
        {
            lock (ThreadSafeLocker) return protocol.ReadAnalogPin(pin);
        }
        public virtual Logic ReadDigitalPin(Int32 pin)                                   //Thread Safe
        {
            lock (ThreadSafeLocker) return protocol.ReadDigitalPin(pin);
        }
        public virtual void SetDigitalPin(Int32 pin, Logic value)                        //Thread Safe
        {
            lock (ThreadSafeLocker) protocol.SetDigitalPin(pin, value);
        }
        public virtual void SetPWMPin(Int32 pin, Int32 value)                            //Thread Safe
        {
            lock (ThreadSafeLocker) protocol.SetPWMPin(pin, value);
        }
        public virtual void SetServoPin(Int32 pin, Int32 value)                          //Thread Safe
        {
            lock (ThreadSafeLocker) protocol.SetServoPin(pin, value);
        }
		public virtual void SetMotorPin(String direction, Int32 pin, Int32 value)         //Thread Safe
        {
            lock (ThreadSafeLocker) protocol.SetMotorPin(direction, pin, value);
        }
        public virtual void SendRaw(List<Byte> data)                                     //Thread Safe
        {
            lock (ThreadSafeLocker) protocol.SendRaw(data);
        }
        public virtual Int32 CallFunction(String functionname)                           //Thread Safe
        {
            lock (ThreadSafeLocker) return protocol.CallFunction(functionname);
        }
        public virtual List<Byte> I2CRead(Byte address, Byte register, Int32 numBytes)   //Thread Safe
        {
            lock (ThreadSafeLocker) return protocol.I2CRead(address, register, numBytes);
        }
        public virtual void I2CWrite(Byte address, Byte register, List<Byte> data)       //Thread Safe
        {
            lock (ThreadSafeLocker) protocol.I2CWrite(address, register, data);
        }

        protected abstract class GenericConnection
        {
            protected Device parent;
            public GenericConnection(Device _parent)
            {
                parent = _parent;
            }

            public abstract void Connect();      //Non-thread Safe
            public abstract void Disconnect();   //Non-thread Safe

            public abstract List<Byte> Send(String _location, List<Byte> _data);     //Non-thread Safe
            protected void AsyncDataStreamReceivedCallback(Byte _data)    //Non-thread Safe (Make sure no concurrent calls, and calls are sequential)
            {
                parent.protocol.DataStreamReceivedCallback(_data);
            }
        }
        protected class SerialConnection : GenericConnection
        {
            private SerialPort _serialPort;

            public SerialConnection(Device _parent) : base(_parent) { }
            public override void Connect()        //Non-thread Safe
            {
                Disconnect();

                // Setup and Open Port
                _serialPort = new SerialPort(parent.GetIdentifier(), 57600, Parity.None, 8, StopBits.One);
                _serialPort.ReadTimeout = 100;
                _serialPort.Open();

                // Clear Buffer
                try { while (true) _serialPort.ReadByte(); } // Clear Buffer
                catch (TimeoutException) { }

                // Attach Reader Listener
                _serialPort.ErrorReceived += _serialPort_ErrorReceived;
                _serialPort.DataReceived += _serialPort_DataReceived;
            }

            public override void Disconnect()     //Non-thread Safe
            {
                try
                {
                    if (_serialPort != null)
                    {
                        if (_serialPort.IsOpen)  _serialPort.Close();
                        _serialPort.DataReceived -= _serialPort_DataReceived;
                        _serialPort.Dispose();
                    }
                }
                catch (Exception) { }
            }

            // Error Handler
            private void _serialPort_ErrorReceived(object sender, SerialErrorReceivedEventArgs e)
            {
                Disconnect();
            }

            // Receiver
            private Object ThreadSafeDataReceivedLocker = new Object();
            private void _serialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
            {
                lock (ThreadSafeDataReceivedLocker)
                {
                    while (_serialPort.BytesToRead > 0)
                        AsyncDataStreamReceivedCallback((Byte)_serialPort.ReadByte());
                }
            }

            // Sender
            public override List<Byte> Send(String _location, List<Byte> _data)     //Non-thread Safe
            {
                // Ignore _location for Serial Port
                try {
                    _serialPort.Write(_data.ToArray(), 0, _data.Count);
                } catch(Exception) {
                    throw new ClassifiedException(Error.Serial_Send_Failed);
                }
                return null;
            }
            
        }
        protected class HTTPConnection : GenericConnection
        {
            public HTTPConnection(Device _parent) : base(_parent) { }
            public override void Connect()        //Non-thread Safe
            {
                // Connectionless
            }
            public override void Disconnect()     //Non-thread Safe
            {
                // Connectionless
            }

            public override List<Byte> Send(String _location, List<Byte> _data)     //Non-thread Safe
            {
                // Ignore _data for HTTP GET request
                return new List<Byte>( new System.Text.UTF8Encoding(true).GetBytes( webRequest(_location) ) );
            }

            private String webRequest(String path)
            {
                WebClient client = new WebClient();
                return client.DownloadString("http://" + path);
            }
        }
        protected class TCPConnection : GenericConnection
        {
            private Socket _tcpSocket;
            private const Int32 _tcpReceiveBufferSize = 1024;
            private Byte[] _tcpReceiveBuffer = new Byte[_tcpReceiveBufferSize];

            public TCPConnection(Device _parent) : base(_parent) { }
            public override void Connect()        //Non-thread Safe
            {
                // Clear Any Old Socket
                Disconnect();

                // Setup TCP Client
                _tcpSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                _tcpSocket.SendBufferSize = 64;
                _tcpSocket.SendTimeout = 2000;
                _tcpSocket.Connect(parent.GetIdentifier(), 1001);
                _tcpSocket.BeginReceive(_tcpReceiveBuffer, 0, _tcpReceiveBufferSize, SocketFlags.None, _tcpSocket_ReadCallback, null);
            }
            public override void Disconnect()     //Non-thread Safe
            {
                try
                {
                    if (_tcpSocket != null)
                    {
                        _tcpSocket.Shutdown(SocketShutdown.Both);
                        _tcpSocket.Close();
                    }
                }
                catch (Exception) { }
            }

            // Receiver
            private Object ThreadSafeDataReceivedLocker = new Object();
            private void _tcpSocket_ReadCallback(IAsyncResult ar)
            {
                lock (ThreadSafeDataReceivedLocker)
                {
                    try
                    {
                        int bytesRead = _tcpSocket.EndReceive(ar);
                        if (bytesRead > 0)
                        {
                            Byte[] buffershadow = new Byte[_tcpReceiveBufferSize];
                            _tcpReceiveBuffer.CopyTo(buffershadow, 0);

                            String buff = Encoding.ASCII.GetString(_tcpReceiveBuffer, 0, bytesRead);
                            _tcpSocket.BeginReceive(_tcpReceiveBuffer, 0, _tcpReceiveBufferSize, SocketFlags.None, _tcpSocket_ReadCallback, null);

                            for (Int32 i = 0; i < bytesRead; i++) AsyncDataStreamReceivedCallback(buffershadow[i]);
                        }
                    }
                    catch (Exception) { }
                }
            }

            // Sender
            public override List<Byte> Send(String _location, List<Byte> _data)     //Non-thread Safe
            {
                // Ignore _location for Serial Port
                SocketError error;
                _tcpSocket.Send(_data.ToArray(), 0, _data.Count, SocketFlags.None, out error);
                if (error != SocketError.Success) throw new ClassifiedException(Error.TCP_Send_Failed);
                return null;
            }
        }
        protected class UDPConnection : GenericConnection
        {
            private UdpClient _udpSocket;
            private IPEndPoint _udpep;

            public UDPConnection(Device _parent) : base(_parent) { }
            public override void Connect()        //Non-thread Safe
            {
                // Setup UDP Transceiver (Connectionless)
                _udpSocket = new UdpClient();
                _udpep = new IPEndPoint(IPAddress.Parse(parent.GetIdentifier()), 1001);
                _udpSocket.Connect(_udpep);
                _udpSocket.BeginReceive(_udpSocket_ReadCallback, null);
            }
            public override void Disconnect()     //Non-thread Safe
            {
                // Remove Transceiver (Connectionless)
                try {
                    _udpSocket.Close();
                } catch (Exception) { }
            }

            // Receiver
            private Object ThreadSafeDataReceivedLocker = new Object();
            private void _udpSocket_ReadCallback(IAsyncResult ar)
            {
                lock (ThreadSafeDataReceivedLocker)
                {
                    try
                    {
                        foreach (Byte b in _udpSocket.EndReceive(ar, ref _udpep))
                            AsyncDataStreamReceivedCallback(b);
                        _udpSocket.BeginReceive(_udpSocket_ReadCallback, null);
                    }
                    catch (Exception) { }
                }
            }

            // Sender
            public override List<Byte> Send(String _location, List<Byte> _data)     //Non-thread Safe
            {
                // Ignore _location for Serial Port
                _udpSocket.Send(_data.ToArray(), _data.Count);
                return null;
            }
        }

        protected abstract class GenericProtocol
        {
            protected Device parent;
            public GenericProtocol(Device _parent)
            {
                parent = _parent;
            }

            public abstract void Connect();      //Non-thread Safe
            public abstract void Disconnect();   //Non-thread Safe

            protected virtual List<Byte> Send(String _location, List<Byte> _data)     //Non-thread Safe
            {
                return parent.connection.Send(_location, _data);
            }
            public abstract void DataStreamReceivedCallback(Byte _incomingbyte);    //Thread Safe (Make sure calls are sequential)

            public abstract void ResetPinState();                                                   //Thread Safe
            public abstract Int32 ReadAnalogPin(Int32 pin);                                         //Thread Safe
            public abstract Logic ReadDigitalPin(Int32 pin);                                        //Thread Safe
            public abstract void SetDigitalPin(Int32 pin, Logic value);                             //Thread Safe
            public abstract void SetPWMPin(Int32 pin, Int32 value);                                 //Thread Safe
            public abstract void SetServoPin(Int32 pin, Int32 value);                               //Thread Safe
            public abstract void SetMotorPin(String direction, Int32 pin, Int32 value);              // Add by Ken 20171122
            public abstract void SendRaw(List<Byte> data);                                          //Thread Safe
            public abstract Int32 CallFunction(String functionname);                                //Thread Safe
            public abstract List<Byte> I2CRead(Byte address, Byte register, Int32 numBytes);        //Thread Safe
            public abstract void I2CWrite(Byte address, Byte register, List<Byte> data);            //Thread Safe
        }
        protected class FirmataProtocol : GenericProtocol
        {
            #region Firmata Buffer Variables
            protected Object BufferThreadSafeLocker = new Object();
            protected volatile Int32 majorVersion = 0;
            protected volatile Int32 minorVersion = 0;
            protected volatile List<PinType> digitalType = new List<PinType>();    // Required as reporting system
            protected volatile List<Logic> digitalOutputData = new List<Logic>();  // Required as version before 2.4 of Firmata needs a group (port) update of Digital Output
            protected volatile List<Logic?> digitalInputData = new List<Logic?>(); // Required as reporting system
            protected volatile List<PinType> analogType = new List<PinType>();     // Required as reporting system
            protected volatile List<Int32?> analogInputData = new List<Int32?>();  // Required as reporting system
            protected volatile List<Byte> incomingBuffer = new List<Byte>();
            protected volatile Dictionary<Byte, Dictionary<Byte, List<Byte>>> I2CReadBuffer = new Dictionary<Byte, Dictionary<Byte, List<Byte>>>(); // [address][register]
            #endregion
            #region Firmata Constants
            // Mask
            protected const Byte CHANNEL_MASK = 0x0F; // channel mask
            protected const Byte CMD_MASK = 0xF0; // cmd mask
            // Basic Command
            protected const Byte DIGITAL_MESSAGE = 0x90; // send data for a digital port
            protected const Byte ANALOG_MESSAGE = 0xE0; // send data for an analog pin (or PWM)
            protected const Byte REPORT_ANALOG = 0xC0; // enable analog input by pin #
            protected const Byte REPORT_DIGITAL = 0xD0; // enable digital input by port
            protected const Byte SET_PIN_MODE = 0xF4; // set a pin to INPUT/OUTPUT/PWM/etc
            protected const Byte REPORT_VERSION = 0xF9; // report firmware version
            protected const Byte SYSTEM_RESET = 0xFF; // reset from MIDI
            protected const Byte START_SYSEX = 0xF0; // start a MIDI SysEx message
            protected const Byte END_SYSEX = 0xF7; // end a MIDI SysEx message
            // SysEx Command
            protected const Byte REPORT_FIRMWARE = 0x79;
            protected const Byte I2C_REQUEST = 0x76;
            protected const Byte I2C_REPLY = 0x77;
            protected const Byte I2C_CONFIG = 0x78;
            protected const Byte STRING_DATA = 0x71;
            #endregion
            #region Firmata Dynamic Configuration
            protected Int32 I2CPort = -1;   // -1 means N/A
            protected Int32 I2CSDA = -1;    // -1 means N/A
            protected Int32 I2CSCL = -1;    // -1 means N/A
            public const Int32 DIGITAL_PORT = 0;
            public const Int32 ANALOG_PORT = 1;
            public void I2C_Setting(Int32 _port, Int32 _sda, Int32 _scl)
            {
                I2CPort = _port;
                I2CSDA = _sda;
                I2CSCL = _scl;
            }
            #endregion

            public FirmataProtocol(Device _parent)
                : base(_parent)
            {
                Int32 requiredNumOfPin = -1;
                foreach (KeyValuePair<PinType, List<Int32>> pintypelist in parent.PinResources) requiredNumOfPin = Math.Max(requiredNumOfPin, pintypelist.Value.Max());

                for (Int32 i = 0; i <= requiredNumOfPin; i++)
                {
                    digitalType.Add(PinType.UNDEFINED);
                    digitalOutputData.Add(Logic.LOW);
                    digitalInputData.Add(null);

                    // May add more buffer than needed
                    analogType.Add(PinType.UNDEFINED);
                    analogInputData.Add(null);
                }
            }
            protected void resetBuffers()
            {
                lock (BufferThreadSafeLocker)
                {
                    for (Int32 i = 0; i < digitalType.Count; i++) digitalType[i] = PinType.UNDEFINED;
                    for (Int32 i = 0; i < digitalOutputData.Count; i++) digitalOutputData[i] = Logic.LOW;
                    for (Int32 i = 0; i < digitalInputData.Count; i++) digitalInputData[i] = null;
                    for (Int32 i = 0; i < analogType.Count; i++) analogType[i] = PinType.UNDEFINED;
                    for (Int32 i = 0; i < analogInputData.Count; i++) analogInputData[i] = null;

                    VersionUpdated.Reset();
                    PinUpdated.Reset();
                    I2CReadUpdated.Reset();

                    incomingBuffer = new List<Byte>();
                }
            }
            public override void Connect()       //Non-thread Safe
            {
                // Reset Buffers & System Reset
                ResetPinState();
                
                // Ask for Firmata Version
                majorVersion = 0;
                minorVersion = 0;
                Send(new List<Byte>() { REPORT_VERSION });

                // Check Firmata Version
                VersionUpdated.WaitOne(3000);
                lock (BufferThreadSafeLocker)
                {
                    if (majorVersion == 0 && minorVersion == 0) throw new ClassifiedException(Error.Firmata_Invalid_Version); // Non-Firmata Detected
                    if (!CheckVersion(2, 3)) throw new ClassifiedException(Error.Firmata_Old_Version); // Old Version Detected
                }
            }
            public override void Disconnect()    //Non-thread Safe
            {
                // [Quiet] System Reset
                try
                {
                    Send(new List<Byte>() { SYSTEM_RESET });
                }
                catch (Exception) { }

                // Reset Buffers
                resetBuffers();
            }

            protected AutoResetEvent VersionUpdated = new AutoResetEvent(false);
            protected AutoResetEvent PinUpdated = new AutoResetEvent(false);
            protected AutoResetEvent I2CReadUpdated = new AutoResetEvent(false);

            protected List<Byte> Send(List<Byte> _data)
            {
                List<Byte> returnedBytes = parent.connection.Send("", _data);
                if (returnedBytes != null) foreach (Byte _byte in returnedBytes) DataStreamReceivedCallback(_byte);
                return null;
            }
            public override void DataStreamReceivedCallback(Byte _incomingbyte)    //Thread Safe (Make sure calls are sequential)
            {
                // Process Buffer
                lock (BufferThreadSafeLocker)
                {
                    incomingBuffer.Add(_incomingbyte);
                    if (incomingBuffer[0] == REPORT_VERSION) {
                        #region REPORT_VERSION
                        if (incomingBuffer.Count >= 3)
                        {
                            lock (BufferThreadSafeLocker)
                            {
                                majorVersion = incomingBuffer[1];
                                minorVersion = incomingBuffer[2];
                            }
                            VersionUpdated.Set();
                            incomingBuffer.Clear();
                        }
                        #endregion
                    } else if (incomingBuffer[0] == START_SYSEX) {
                        #region START_SYSEX
                        if (incomingBuffer[incomingBuffer.Count - 1] == END_SYSEX) {
                            if (incomingBuffer[1] == REPORT_FIRMWARE) {
                                // Detect as system reset, report error if any pin changed mode
                                if (digitalType.Exists(x => x != PinType.UNDEFINED) || analogType.Exists(x => x != PinType.UNDEFINED)) {
                                    DeviceManager.DeviceResetCallback(parent);
                                }
                                resetBuffers(); // Mainly Prevent I2C Read Without Setup (which would hang the Firmata)
                            } else if (incomingBuffer[1] == I2C_REPLY) {
                                // I2C Reply
                                Byte address = (Byte)((incomingBuffer[2] & 0x7F) | ((incomingBuffer[3] & 0x7F) << 7));
                                Byte register = (Byte)((incomingBuffer[4] & 0x7F) | ((incomingBuffer[5] & 0x7F) << 7));
                                if (I2CReadBuffer[address][register] == null) {
                                    List<Byte> data = new List<Byte>();
                                    for (Int32 i = 7; i < (incomingBuffer.Count - 1); i += 2) data.Add((Byte)((incomingBuffer[i - 1] & 0x7F) | ((incomingBuffer[i] & 0x7F) << 7)));
                                    if (I2CReadBuffer.ContainsKey(address)) if (I2CReadBuffer[address].ContainsKey(register)) I2CReadBuffer[address][register] = data;
                                }
                                I2CReadUpdated.Set();
                            } else if (incomingBuffer[1] == STRING_DATA) {
                                if (incomingBuffer[2] == 'I' && incomingBuffer[3] == 0x00 && incomingBuffer[4] == '2' && incomingBuffer[5] == 0x00 && incomingBuffer[6] == 'C' && incomingBuffer[7] == 0x00)
                                { //I2C Error
                                    //Set Buffer to Empty Return
                                    List<Byte> addresses = new List<Byte>(I2CReadBuffer.Keys);
                                    foreach (Byte address in addresses) {
                                        List<Byte> registers = new List<Byte>(I2CReadBuffer[address].Keys);
                                        foreach (Byte register in registers) {
                                            if (I2CReadBuffer[address][register] == null)
                                                I2CReadBuffer[address][register] = new List<Byte>();
                                        }
                                    }
                                    //Update
                                    I2CReadUpdated.Set();
                                }
                            }
                            incomingBuffer.Clear();
                        }
                        #endregion
                    } else if ((incomingBuffer[0] & CMD_MASK) == DIGITAL_MESSAGE) {
                        #region DIGITAL_MESSAGE
                        if (incomingBuffer.Count >= 3)
                        {
                            Int32 pinport = incomingBuffer[0] & CHANNEL_MASK;
                            for (Int32 i = 0; (i < 8) && ((pinport * 8 + i) < digitalType.Count); i++) {
                                if (digitalType[pinport * 8 + i] != PinType.INPUT) continue;
                                Logic newvalue = LogicHelper.Parse(((incomingBuffer[i == 7 ? 2 : 1] >> (i == 7 ? 0 : i)) & 0x01) == 0x01);
                                Boolean setupdate = (digitalInputData[pinport * 8 + i] == null);
                                // if (digitalInputData[pinport * 8 + i] != newvalue && digitalInputData[pinport * 8 + i] != null) DeviceManager.DeviceDigitalReadCallback(parent, pinport * 8 + i, newvalue);
                                digitalInputData[pinport * 8 + i] = newvalue;
                                if (setupdate) PinUpdated.Set();
                            }
                            incomingBuffer.Clear();
                        }
                        #endregion
                    } else if ((incomingBuffer[0] & CMD_MASK) == ANALOG_MESSAGE) {
                        #region ANALOG_MESSAGE
                        if (incomingBuffer.Count >= 3)
                        {
                            Int32 pin = incomingBuffer[0] & CHANNEL_MASK;
                            if (analogType[pin] == PinType.ANALOG)
                            {
                                Boolean setupdate = (analogInputData[pin] == null);
                                analogInputData[pin] = ((incomingBuffer[2] & 0x7F) << 7) | (incomingBuffer[1] & 0x7F);
                                if (setupdate) PinUpdated.Set();
                            }
                            incomingBuffer.Clear();
                        }
                        #endregion
                    } else {
                        #region Unknown Message
                        incomingBuffer.Clear();
                        #endregion
                    }
                }
            }

            protected Boolean CheckVersion(Int32 minMajor, Int32 minMinor)
            {
                if (majorVersion < minMajor) return false;
                if (majorVersion == minMajor && minorVersion < minMinor) return false;
                return true;
            }
            protected Boolean CheckPinValid(PinType type, Int32 pin)
            {
                if (type == PinType.UNDEFINED) return false;
                if (!parent.PinResources.ContainsKey(type)) return false;
                if (!parent.PinResources[type].Contains(pin)) return false;
                return true;
            }
            protected Int32 GetPinTypeValue(PinType type)
            {
                switch(type)
                {
                    case PinType.INPUT:     return 0;
                    case PinType.OUTPUT:    return 1;
                    case PinType.ANALOG:    return 2;
                    case PinType.PWM:       return 3;
                    case PinType.SERVO:     return 4;
                    case PinType.I2C:       return 6;
                    case PinType.ONEWIRE:   return 7;
                    case PinType.STEPPER:   return 8;
                    case PinType.ENCODER:   return 9;
                }
                throw new ClassifiedException(Error.Compile_Time_Mistake_PinTypeUnsupported); // Un-supported by Firmata
            }
            protected void ResetDigitalReportCheck(Int32 changedpin)
            {
                Boolean noinput = true;
                Int32 pinport = changedpin / 8;
                for (Int32 i = (pinport * 8); (i < (pinport + 1) * 8) && (i < digitalType.Count) && noinput; i++) if (digitalType[i] == PinType.INPUT) noinput = false;
                if (noinput) Send(new List<Byte>() { (Byte)(REPORT_DIGITAL | pinport), 0 });
            }
            protected void ResetAnalogReportCheck(Int32 changedpin)
            {
                if (analogType[changedpin] == PinType.ANALOG) Send(new List<Byte>() { (Byte)(REPORT_ANALOG | changedpin), 0 });
            }

            public override void ResetPinState()                                                //Thread Safe
            {
                lock (BufferThreadSafeLocker)
                {
                    resetBuffers();
                    Send(new List<Byte>() { SYSTEM_RESET });
                }
            }
            public override Int32 ReadAnalogPin(Int32 pin)                                      //Thread Safe
            {
                if (!CheckPinValid(PinType.ANALOG, pin)) throw new ClassifiedException(Error.Input_PinType_Mismatch);

                lock (BufferThreadSafeLocker)
                {
                    // Set Report Analog Pin
                    Boolean alreadSet = false;
                    if (analogType[pin] == PinType.ANALOG) alreadSet = true;
                    if (!alreadSet)
                    {
                        Send(new List<Byte>() { (Byte)(REPORT_ANALOG | pin), 1 });
                        analogType[pin] = PinType.ANALOG;
                        analogInputData[pin] = null;
                    }

                    // Report if Version older than v2.4
                    if (!CheckVersion(2, 4)) return (analogInputData[pin] ?? 0);

                    // Report if Already Reporting
                    if (alreadSet && analogInputData[pin] != null) return (analogInputData[pin] ?? 0);
                }

                // Wait Until Reply
                DateTime loopStart = DateTime.Now;
                while (true)
                {
                    if ((DateTime.Now - loopStart).TotalMilliseconds > 3000) throw new ClassifiedException(Error.Firmata_Read_NoRespond);
                    PinUpdated.WaitOne(500);
                    lock (BufferThreadSafeLocker) if (analogInputData[pin] != null) return (analogInputData[pin] ?? 0);
                }
            }
            public override Logic ReadDigitalPin(Int32 pin)                                     //Thread Safe
            {
                if (!CheckPinValid(PinType.INPUT, pin)) throw new ClassifiedException(Error.Input_PinType_Mismatch);

                lock (BufferThreadSafeLocker)
                {
                    // Set Report Digital Port
                    Boolean alreadSet = false;
                    Int32 pinport = pin / 8;
                    for (Int32 i = (pinport * 8); (i < (pinport + 1) * 8) && (i < digitalType.Count) && !alreadSet; i++) if (digitalType[i] == PinType.INPUT) alreadSet = true;
                    if (!alreadSet)  Send(new List<Byte>() { (Byte)(REPORT_DIGITAL | pinport), 1 });

                    // Set Pin Mode
                    if (digitalType[pin] != PinType.INPUT)
                    {
                        Send(new List<Byte>() { SET_PIN_MODE, (Byte)pin, (Byte)GetPinTypeValue(PinType.INPUT) });
                        digitalType[pin] = PinType.INPUT;
                        digitalInputData[pin] = null; // Reset Buffer
                        digitalOutputData[pin] = Logic.LOW; // Reset Buffer
                    }

                    // Report if Version older than v2.4
                    if (!CheckVersion(2, 4)) return (digitalInputData[pin] ?? Logic.LOW);

                    // Report if Already Reporting
                    if (alreadSet && digitalInputData[pin] != null) return (digitalInputData[pin] ?? Logic.LOW);
                }

                // Wait Until Reply
                DateTime loopStart = DateTime.Now;
                while (true)
                {
                    if ((DateTime.Now - loopStart).TotalMilliseconds > 3000) throw new ClassifiedException(Error.Firmata_Read_NoRespond);
                    PinUpdated.WaitOne(500);
                    lock (BufferThreadSafeLocker) if (digitalInputData[pin] != null) return (digitalInputData[pin] ?? Logic.LOW);
                }
            }
            public override void SetDigitalPin(Int32 pin, Logic value)                          //Thread Safe
            {
                if (!CheckPinValid(PinType.OUTPUT, pin)) throw new ClassifiedException(Error.Input_PinType_Mismatch);
                lock (BufferThreadSafeLocker)
                {
                    // Set Pin Mode
                    if (digitalType[pin] != PinType.OUTPUT)
                    {
                        digitalType[pin] = PinType.OUTPUT;
                        digitalInputData[pin] = null; // Reset Buffer
                        ResetDigitalReportCheck(pin); // Reset Buffer
                        Send(new List<Byte>() { SET_PIN_MODE, (Byte)pin, (Byte)GetPinTypeValue(PinType.OUTPUT) });
                    }

                    // Buffer Pin Value
                    digitalOutputData[pin] = value;

                    // Set Port to Digital Value
                    Int32 pinport = pin / 8;
                    Byte lowerByte = 0x00;
                    for (Int32 i = 0; (i < 7) && ((pinport * 8 + i) < digitalType.Count); i++) lowerByte |= (Byte)(LogicHelper.ToByte(digitalOutputData[pinport * 8 + i]) << i);
                    Send(new List<Byte>() { ((Byte)(DIGITAL_MESSAGE | pinport)), lowerByte, (((pinport * 8 + 7) < digitalType.Count) ? (LogicHelper.ToByte(digitalOutputData[pinport * 8 + 7])) : ((Byte)0x00)) });
                }
            }
            public override void SetPWMPin(Int32 pin, Int32 value)                              //Thread Safe
            {
                if (!CheckPinValid(PinType.PWM, pin)) throw new ClassifiedException(Error.Input_PinType_Mismatch);
                lock (BufferThreadSafeLocker)
                {
                    // Set Pin Mode
                    if (digitalType[pin] != PinType.PWM)
                    {
                        digitalType[pin] = PinType.PWM;
                        digitalInputData[pin] = null; // Reset Buffer
                        digitalOutputData[pin] = Logic.LOW; // Reset Buffer
                        ResetDigitalReportCheck(pin); // Reset Buffer
                        Send(new List<Byte>() { SET_PIN_MODE, (Byte)pin, (Byte)GetPinTypeValue(PinType.PWM) });
                    }

                    // Set Pin to PWM Value
                    value = (Int32)Math.Max(Math.Min(value, 255), 0);
                    Send(new List<Byte>() { (Byte)(ANALOG_MESSAGE + pin), (Byte)(value & 0x7F), (Byte)((value >> 7) & 0x7F) });
                }
            }
            public override void SetServoPin(Int32 pin, Int32 value)                            //Thread Safe
            {
                if (!CheckPinValid(PinType.SERVO, pin)) throw new ClassifiedException(Error.Input_PinType_Mismatch);

                lock (BufferThreadSafeLocker)
                {
                    // Set Pin Mode
                    if (digitalType[pin] != PinType.SERVO)
                    {
                        digitalType[pin] = PinType.SERVO;
                        digitalInputData[pin] = null; // Reset Buffer
                        digitalOutputData[pin] = Logic.LOW; // Reset Buffer
                        ResetDigitalReportCheck(pin); // Reset Buffer
                        Send(new List<Byte>() { SET_PIN_MODE, (Byte)pin, (Byte)GetPinTypeValue(PinType.SERVO) });
                    }

                    // Set Pin to Servo Value
                    value = (Int32)Math.Max(Math.Min(value, 180), 0);
                    Send(new List<Byte>() { (Byte)(ANALOG_MESSAGE + pin), (Byte)(value & 0x7F), (Byte)((value >> 7) & 0x7F) });
                }
            }
			public override void SetMotorPin(String direction, Int32 pin, Int32 value)                          //Thread Safe
            {
                if (!CheckPinValid(PinType.SERVO, pin)) throw new ClassifiedException(Error.Input_PinType_Mismatch);

                lock (BufferThreadSafeLocker)
                {
                    // Set Pin Mode
                    if (digitalType[pin] != PinType.SERVO)
                    {
                        digitalType[pin] = PinType.SERVO;
                        digitalInputData[pin] = null; // Reset Buffer
                        digitalOutputData[pin] = Logic.LOW; // Reset Buffer
                        ResetDigitalReportCheck(pin); // Reset Buffer
                        Send(new List<Byte>() { SET_PIN_MODE, (Byte)pin, (Byte)GetPinTypeValue(PinType.SERVO) });
                    }

                    // Set Pin to Servo Value
                    value = (Int32)Math.Max(Math.Min(value, 180), 0);
                    Send(new List<Byte>() { (Byte)(ANALOG_MESSAGE + pin), (Byte)(value & 0x7F), (Byte)((value >> 7) & 0x7F) });
                }
            }
            public override void SendRaw(List<byte> data)                                       //Thread Safe
            {
                lock (BufferThreadSafeLocker)
                {
                    Send(data);
                }
            }
            public override Int32 CallFunction(String functionname)                             //Thread Safe
            {
                if (!parent.FunctionResources.Contains(functionname)) throw new ClassifiedException(Error.Input_FunctionName_Mismatch);

                // Not Implemented
                throw new ClassifiedException(Error.Compile_Time_Mistake_FunctionUnimplemented);
            }
            public override List<Byte> I2CRead(Byte address, Byte register, Int32 numBytes)     //Thread Safe
            {
                if (I2CPort == -1 || I2CSDA == -1 || I2CSCL == -1) throw new ClassifiedException(Error.Input_I2C_NotSupported);
                if (numBytes <= 0) return new List<Byte>(); // Return 0 Bytes immediately if 0 bytes requested

                lock (BufferThreadSafeLocker)
                {
                    // I2C Config and Pin State
                    if (I2CPort == DIGITAL_PORT)
                    {
                        if (digitalType[I2CSDA] != PinType.I2C || digitalType[I2CSCL] != PinType.I2C)
                        {
                            digitalType[I2CSDA] = PinType.I2C;
                            digitalInputData[I2CSDA] = null; // Reset Buffer
                            digitalOutputData[I2CSDA] = Logic.LOW; // Reset Buffer
                            ResetDigitalReportCheck(I2CSDA); // Reset Buffer

                            digitalType[I2CSCL] = PinType.I2C;
                            digitalInputData[I2CSCL] = null; // Reset Buffer
                            digitalOutputData[I2CSCL] = Logic.LOW; // Reset Buffer
                            ResetDigitalReportCheck(I2CSCL); // Reset Buffer

                            // Init I2C with read delay time setup where data = [read_delay_time & 0x7f, read_delay_time >> 7]
                            Send(new List<Byte>() { START_SYSEX, I2C_CONFIG, 0x00, 0x00, END_SYSEX });
                        }
                    }
                    else
                    {
                        if (analogType[I2CSDA] != PinType.I2C || analogType[I2CSCL] != PinType.I2C)
                        {
                            analogType[I2CSDA] = PinType.I2C;
                            analogInputData[I2CSDA] = null; // Reset Buffer
                            ResetAnalogReportCheck(I2CSDA);

                            analogType[I2CSCL] = PinType.I2C;
                            analogInputData[I2CSCL] = null; // Reset Buffer
                            ResetAnalogReportCheck(I2CSCL);

                            // Init I2C with read delay time setup where data = [read_delay_time & 0x7f, read_delay_time >> 7]
                            Send(new List<Byte>() { START_SYSEX, I2C_CONFIG, 0x00, 0x00, END_SYSEX });
                        }
                    }
                    
                    // Prepare Buffer
                    if (!I2CReadBuffer.ContainsKey(address))
                        I2CReadBuffer[address] = new Dictionary<Byte,List<Byte>>();
                    if (!I2CReadBuffer[address].ContainsKey(register))
                        I2CReadBuffer[address].Add(register, null);
                    else
                        I2CReadBuffer[address][register] = null;

                    // I2C Request
                    I2CReadUpdated.Reset();
                    Send(new List<Byte>() { START_SYSEX, I2C_REQUEST, (Byte)(address & 0x7F), 0x48, (Byte)(register & 0x7F), (Byte)(register >> 7), (Byte)(numBytes & 0x7F), (Byte)((numBytes >> 7) & 0x7F), END_SYSEX });
                }

                // Wait Until Reply
                DateTime loopStart = DateTime.Now;
                while (true)
                {
                    if ((DateTime.Now - loopStart).TotalMilliseconds > 3000) throw new ClassifiedException(Error.Firmata_Read_NoRespond);
                    I2CReadUpdated.WaitOne(500);
                    lock (BufferThreadSafeLocker) if (I2CReadBuffer[address][register] != null)
                    {
                        if (I2CReadBuffer[address][register].Count == 0) throw new ClassifiedException(Error.Firmata_I2C_Error);
                        return I2CReadBuffer[address][register];
                    }
                }
            }
            public override void I2CWrite(Byte address, Byte register, List<Byte> data)         //Thread Safe
            {
                if (I2CPort == -1 || I2CSDA == -1 || I2CSCL == -1) throw new ClassifiedException(Error.Input_I2C_NotSupported);

                lock (BufferThreadSafeLocker)
                {
                    // Not Implemented
                    throw new ClassifiedException(Error.Compile_Time_Mistake_Unimplemented);
                    // TODO: Implement I2C Write
                }
            }
        }
        protected class ARestSimpleProtocol : GenericProtocol
        {
            #region ARest Buffer Variables
            protected Object BufferThreadSafeLocker = new Object();
            protected volatile List<PinType> digitalType = new List<PinType>();    // Required as reporting system
            protected volatile List<PinType> analogType = new List<PinType>();     // Required as reporting system
            #endregion

            public ARestSimpleProtocol(Device _parent)
                : base(_parent)
            {
                Int32 requiredNumOfPin = -1;
                foreach (KeyValuePair<PinType, List<Int32>> pintypelist in parent.PinResources) requiredNumOfPin = Math.Max(requiredNumOfPin, pintypelist.Value.Max());

                for (Int32 i = 0; i <= requiredNumOfPin; i++)
                {
                    // May add more buffer than needed
                    digitalType.Add(PinType.UNDEFINED);
                    analogType.Add(PinType.UNDEFINED);
                }

                // Decode IP Address and Password
                String standardID = _parent.GetIdentifier().Replace(" ", "").Replace("-", "").ToUpper().Replace('O', '0').Replace('I', '1').Replace('L', '1').Replace('Q', '9');
                Int32 password_value = Convert.ToInt32(standardID.Substring(4, 2), 16);
                ipaddr = DeviceManager.interfaceIPAddress.GetAddressBytes()[0].ToString();
                ipaddr += "." + DeviceManager.interfaceIPAddress.GetAddressBytes()[1].ToString();
                ipaddr += "." + (Convert.ToInt32(standardID.Substring(0, 2), 16) ^ password_value).ToString();
                ipaddr += "." + (Convert.ToInt32(standardID.Substring(2, 2), 16) ^ password_value).ToString();
                password = password_value.ToString();
            }
            protected void resetBuffers()
            {
                lock (BufferThreadSafeLocker)
                {
                    for (Int32 i = 0; i < digitalType.Count; i++) digitalType[i] = PinType.UNDEFINED;
                    for (Int32 i = 0; i < analogType.Count; i++) analogType[i] = PinType.UNDEFINED;
                }
            }
            public override void Connect()
            {
                ResetPinState();
            }
            public override void Disconnect()
            {
                resetBuffers();
            }

            private String ipaddr = "127.0.0.1";
            private String password = "";

            private String Send(String _location)
            {
                ASCIIEncoding encoding = new ASCIIEncoding();
                try {
                    return encoding.GetString(parent.connection.Send(ipaddr + _location + "/" + password, null).ToArray());
                } catch (Exception) {
                    throw new ClassifiedException(Error.aRest_ConnectionLost);
                }
            }
            public override void DataStreamReceivedCallback(Byte _incomingbyte)
            {
                // No Callback for aRest
            }
            private Dictionary<String, String> jsonParser(String json)
            {
                Dictionary<String, String> returnDict = new Dictionary<String, String>();
                json = json.Replace("\r", "").Replace("\n", "");
                if(json.Length < 4) return returnDict;

                String[] items = json.Substring(2, json.Length - 4).Split(new String[] { "\": \"", "\", \"" }, StringSplitOptions.None);
                for (Int32 i = 0; (i + 1) < items.Length; i += 2) returnDict.Add(items[i], items[i + 1]);

                return returnDict;
            }

            protected Boolean CheckPinValid(PinType type, Int32 pin)
            {
                if (type == PinType.UNDEFINED) return false;
                if (!parent.PinResources.ContainsKey(type)) return false;
                if (!parent.PinResources[type].Contains(pin)) return false;
                return true;
            }

            public override void ResetPinState()                                                //Thread Safe
            {
                lock (BufferThreadSafeLocker)
                {
                    resetBuffers();
                    Send("/pwm/1/0");
                    Send("/pwm/8/0");
                    Send("/motor/1/cw/0");
                    Send("/motor/2/cw/0");
                }
            }
            public override Int32 ReadAnalogPin(Int32 pin)                                      //Thread Safe
            {
                if (!CheckPinValid(PinType.ANALOG, pin)) throw new ClassifiedException(Error.Input_PinType_Mismatch);

                lock (BufferThreadSafeLocker)
                {
                    analogType[pin] = PinType.ANALOG;

                    Int32 result = Int32.Parse(Send("/analog/" + pin.ToString() + "/r"));
                    if (result == 65535) throw new ClassifiedException(Error.aRest_ADCError);
                    return result;
                }
            }
            public override Logic ReadDigitalPin(Int32 pin)                                     //Thread Safe
            {
                if (!CheckPinValid(PinType.INPUT, pin)) throw new ClassifiedException(Error.Input_PinType_Mismatch);

                lock (BufferThreadSafeLocker)
                {
                    // Set Pin Mode
                    if (digitalType[pin] != PinType.INPUT)
                    {
                        Send("/mode/" + pin.ToString() + "/i");
                        digitalType[pin] = PinType.INPUT;
                    }

                    // Set Output
                    Dictionary<String, String> respond = jsonParser(Send("/digital/" + pin.ToString() + "/r"));
                    return LogicHelper.Parse(respond["return_value"]);
                }
            }
            public override void SetDigitalPin(Int32 pin, Logic value)                          //Thread Safe
            {
                if (!CheckPinValid(PinType.OUTPUT, pin)) throw new ClassifiedException(Error.Input_PinType_Mismatch);

                lock (BufferThreadSafeLocker)
                {
                    // Set Pin Mode
                    if (digitalType[pin] != PinType.OUTPUT)
                    {
                        Send("/mode/" + pin.ToString() + "/o");
                        digitalType[pin] = PinType.OUTPUT;
                    }

                    // Set Output
                    Send("/digital/" + pin.ToString() + "/" + LogicHelper.ToString(value));
                }
            }
            public override void SetPWMPin(Int32 pin, Int32 value)                              //Thread Safe
            {
                if (!CheckPinValid(PinType.PWM, pin)) throw new ClassifiedException(Error.Input_PinType_Mismatch);

                lock (BufferThreadSafeLocker)
                {
                    // Set Pin Mode
                    if (digitalType[pin] != PinType.PWM)
                    {
                        Send("/mode/" + pin.ToString() + "/p");
                        digitalType[pin] = PinType.PWM;
                    }

                    // Set Output
                    Send("/pwm/" + pin.ToString() + "/" + value.ToString());
                }
            }
            public override void SetServoPin(Int32 pin, Int32 value)                            //Thread Safe
            {
                if (!CheckPinValid(PinType.SERVO, pin)) throw new ClassifiedException(Error.Input_PinType_Mismatch);

                lock (BufferThreadSafeLocker)
                {
                    digitalType[pin] = PinType.SERVO;

                    // Set Output
                    Send("/servo/" + pin.ToString() + "/" + value.ToString());
                }
            }
			public override void SetMotorPin(String direction, Int32 pin, Int32 value)                            //Thread Safe
            {
                if (!CheckPinValid(PinType.MOTOR, pin)) throw new ClassifiedException(Error.Input_PinType_Mismatch);

                lock (BufferThreadSafeLocker)
                {
                    digitalType[pin] = PinType.MOTOR;

                    // Set Output
                    Send("/motor/"+ pin.ToString() + "/" + direction.ToString() + "/" + value.ToString());
                }
            }
            public override void SendRaw(List<byte> data)                                       //Thread Safe
            {
                throw new ClassifiedException(Error.Compile_Time_Mistake_Unimplemented);
            }
            public override Int32 CallFunction(String functionname)                             //Thread Safe
            {
                if (!parent.FunctionResources.Contains(functionname)) throw new ClassifiedException(Error.Input_FunctionName_Mismatch);

                Int32 returnValue = 0;
                switch (functionname)
                {
                    case "motor-forward":
                        Send("/forward/1023");
                        break;
                    case "motor-backward":
                        Send("/backward/1023");
                        break;
                    case "motor-left":
                        Send("/left/1023");
                        break;
                    case "motor-right":
                        Send("/right/1023");
                        break;
                    case "motor-stop":
                        Send("/stop");
                        break;

                    // Not Implemented
                    default:
                        throw new ClassifiedException(Error.Compile_Time_Mistake_FunctionUnimplemented);
                }

                return returnValue;
            }
            public override List<Byte> I2CRead(Byte address, Byte register, Int32 numBytes)     //Thread Safe
            {
                lock (BufferThreadSafeLocker)
                {
                    String url = "/i2cread/";
                    url += ((Int32)address).ToString() + "_" + ((Int32)register).ToString() + "/";
                    url += numBytes.ToString();
                    Dictionary<String, String> respond = jsonParser(Send(url));
                    String[] returndataStr = respond["return_value"].Split('_');
                    if (returndataStr.Length != numBytes) throw new ClassifiedException(Error.aRest_I2C_Read_NumOfByteMismatch);

                    List<Byte> returndataByte = new List<Byte>();
                    foreach (String _byte in returndataStr) returndataByte.Add((Byte)Int32.Parse(_byte));
                    return returndataByte;
                }
            }
            public override void I2CWrite(Byte address, Byte register, List<Byte> data)         //Thread Safe
            {
                lock (BufferThreadSafeLocker)
                {
                    String url = "/rgb/0x40/";
                    List<String> tempdata = new List<String>();
                    foreach (Byte _byte in data) tempdata.Add(((Int32)_byte).ToString());
                    url += String.Join("/", tempdata.ToArray());
                    Send(url);
                }
            }
        }
        protected class ARestIPProtocol : GenericProtocol
        {
            #region ARest Buffer Variables
            protected Object BufferThreadSafeLocker = new Object();
            protected volatile List<PinType> digitalType = new List<PinType>();    // Required as reporting system
            protected volatile List<PinType> analogType = new List<PinType>();     // Required as reporting system
            #endregion

            public ARestIPProtocol(Device _parent)
                : base(_parent)
            {
                Int32 requiredNumOfPin = -1;
                foreach (KeyValuePair<PinType, List<Int32>> pintypelist in parent.PinResources) requiredNumOfPin = Math.Max(requiredNumOfPin, pintypelist.Value.Max());

                for (Int32 i = 0; i <= requiredNumOfPin; i++)
                {
                    // May add more buffer than needed
                    digitalType.Add(PinType.UNDEFINED);
                    analogType.Add(PinType.UNDEFINED);
                }

                // Decode IP Address and Password
                ipaddr = _parent.GetIdentifier();
            }
            protected void resetBuffers()
            {
                lock (BufferThreadSafeLocker)
                {
                    for (Int32 i = 0; i < digitalType.Count; i++) digitalType[i] = PinType.UNDEFINED;
                    for (Int32 i = 0; i < analogType.Count; i++) analogType[i] = PinType.UNDEFINED;
                }
            }
            public override void Connect()
            {
                ResetPinState();
            }
            public override void Disconnect()
            {
                resetBuffers();
            }

            private String ipaddr = "127.0.0.1";

            private String Send(String _location)
            {
                ASCIIEncoding encoding = new ASCIIEncoding();
                try
                {
                    return encoding.GetString(parent.connection.Send(ipaddr + _location, null).ToArray());
                }
                catch (Exception)
                {
                    throw new ClassifiedException(Error.aRest_ConnectionLost);
                }
            }
            public override void DataStreamReceivedCallback(Byte _incomingbyte)
            {
                // No Callback for aRest
            }
            private Dictionary<String, String> jsonParser(String json)
            {
                Dictionary<String, String> returnDict = new Dictionary<String, String>();
                json = json.Replace("\r", "").Replace("\n", "");
                if (json.Length < 4) return returnDict;

                String[] items = json.Substring(2, json.Length - 4).Split(new String[] { "\": \"", "\", \"" }, StringSplitOptions.None);
                for (Int32 i = 0; (i + 1) < items.Length; i += 2) returnDict.Add(items[i], items[i + 1]);

                return returnDict;
            }

            protected Boolean CheckPinValid(PinType type, Int32 pin)
            {
                if (type == PinType.UNDEFINED) return false;
                if (!parent.PinResources.ContainsKey(type)) return false;
                if (!parent.PinResources[type].Contains(pin)) return false;
                return true;
            }

            public override void ResetPinState()                                                //Thread Safe
            {
                lock (BufferThreadSafeLocker)
                {
                    resetBuffers();
                    Send("/pwm/1/0");
                    Send("/pwm/8/0");
                    Send("/motor/1/cw/0");
                    Send("/motor/2/cw/0");
                }
            }
            public override Int32 ReadAnalogPin(Int32 pin)                                      //Thread Safe
            {
                if (!CheckPinValid(PinType.ANALOG, pin)) throw new ClassifiedException(Error.Input_PinType_Mismatch);

                lock (BufferThreadSafeLocker)
                {
                    analogType[pin] = PinType.ANALOG;

                    Int32 result = Int32.Parse(Send("/analog/" + pin.ToString() + "/r"));
                    if (result == 65535) throw new ClassifiedException(Error.aRest_ADCError);
                    return result;
                }
            }
            public override Logic ReadDigitalPin(Int32 pin)                                     //Thread Safe
            {
                if (!CheckPinValid(PinType.INPUT, pin)) throw new ClassifiedException(Error.Input_PinType_Mismatch);

                lock (BufferThreadSafeLocker)
                {
                    // Set Pin Mode
                    if (digitalType[pin] != PinType.INPUT)
                    {
                        Send("/mode/" + pin.ToString() + "/i");
                        digitalType[pin] = PinType.INPUT;
                    }

                    // Set Output
                    Dictionary<String, String> respond = jsonParser(Send("/digital/" + pin.ToString() + "/r"));
                    return LogicHelper.Parse(respond["return_value"]);
                }
            }
            public override void SetDigitalPin(Int32 pin, Logic value)                          //Thread Safe
            {
                if (!CheckPinValid(PinType.OUTPUT, pin)) throw new ClassifiedException(Error.Input_PinType_Mismatch);

                lock (BufferThreadSafeLocker)
                {
                    // Set Pin Mode
                    if (digitalType[pin] != PinType.OUTPUT)
                    {
                        Send("/mode/" + pin.ToString() + "/o");
                        digitalType[pin] = PinType.OUTPUT;
                    }

                    // Set Output
                    Send("/digital/" + pin.ToString() + "/" + LogicHelper.ToString(value));
                }
            }
            public override void SetPWMPin(Int32 pin, Int32 value)                              //Thread Safe
            {
                if (!CheckPinValid(PinType.PWM, pin)) throw new ClassifiedException(Error.Input_PinType_Mismatch);

                lock (BufferThreadSafeLocker)
                {
                    // Set Pin Mode
                    if (digitalType[pin] != PinType.PWM)
                    {
                        Send("/mode/" + pin.ToString() + "/p");
                        digitalType[pin] = PinType.PWM;
                    }

                    // Set Output
                    Send("/pwm/" + pin.ToString() + "/" + value.ToString());
                }
            }
            public override void SetServoPin(Int32 pin, Int32 value)                            //Thread Safe
            {
                if (!CheckPinValid(PinType.SERVO, pin)) throw new ClassifiedException(Error.Input_PinType_Mismatch);

                lock (BufferThreadSafeLocker)
                {
                    digitalType[pin] = PinType.SERVO;

                    // Set Output
                    Send("/servo/" + pin.ToString() + "/" + value.ToString());
                }
            }
            public override void SetMotorPin(String direction, Int32 pin, Int32 value)                            //Thread Safe
            {
                //if (!CheckPinValid(PinType.MOTOR, pin)) throw new ClassifiedException(Error.Input_PinType_Mismatch);

                lock (BufferThreadSafeLocker)
                {
                    digitalType[pin] = PinType.MOTOR;

                    // Set Output
                    Send("/motor/"+ pin.ToString() + "/" + direction.ToString() + "/" + value.ToString());
                }
            }
            public override void SendRaw(List<byte> data)                                       //Thread Safe
            {
                throw new ClassifiedException(Error.Compile_Time_Mistake_Unimplemented);
            }
            public override Int32 CallFunction(String functionname)                             //Thread Safe
            {
                if (!parent.FunctionResources.Contains(functionname)) throw new ClassifiedException(Error.Input_FunctionName_Mismatch);

                Int32 returnValue = 0;
                switch (functionname)
                {
                    case "motor-forward":
                        Send("/forward");
                        break;
                    case "motor-backward":
                        Send("/backward");
                        break;
                    case "motor-left":
                        Send("/left");
                        break;
                    case "motor-right":
                        Send("/right");
                        break;
                    case "motor-stop":
                        Send("/stop");
                        break;

                    // Not Implemented
                    default:
                        throw new ClassifiedException(Error.Compile_Time_Mistake_FunctionUnimplemented);
                }

                return returnValue;
            }
            public override List<Byte> I2CRead(Byte address, Byte register, Int32 numBytes)     //Thread Safe
            {
                lock (BufferThreadSafeLocker)
                {
                    String url = "/i2cread/";
                    url += ((Int32)address).ToString() + "_" + ((Int32)register).ToString() + "/";
                    url += numBytes.ToString();
                    Dictionary<String, String> respond = jsonParser(Send(url));
                    String[] returndataStr = respond["return_value"].Split('_');
                    if (returndataStr.Length != numBytes) throw new ClassifiedException(Error.aRest_I2C_Read_NumOfByteMismatch);

                    List<Byte> returndataByte = new List<Byte>();
                    foreach (String _byte in returndataStr) returndataByte.Add((Byte)Int32.Parse(_byte));
                    return returndataByte;
                }
            }
            public override void I2CWrite(Byte address, Byte register, List<Byte> data)         //Thread Safe
            {
                lock (BufferThreadSafeLocker)
                {
                    String url = "/rgb/0x40/";
                    List<String> tempdata = new List<String>();
                    foreach (Byte _byte in data) tempdata.Add(((Int32)_byte).ToString());
                    url += String.Join("/", tempdata.ToArray());
                    Send(url);
                }
            }
        }
    }


    class ArduinoUnoSerialDevice : Device
    {
        public static String Description = "Arduino UNO";

        public ArduinoUnoSerialDevice(String _identifier)
            : base(_identifier)
        {
            // Device Config
            PinResources.Add(PinType.INPUT,  new List<Int32>() { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 });      //Pin 13 is not recommended to use as INPUT
            PinResources.Add(PinType.OUTPUT, new List<Int32>() { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 });  //Pin 0 and Pin 1 cannot be used (UART)
            PinResources.Add(PinType.ANALOG, new List<Int32>() { 0, 1, 2, 3, 4, 5 });
            PinResources.Add(PinType.PWM,    new List<Int32>() { 3, 5, 6, 9, 10, 11 });
            PinResources.Add(PinType.SERVO,  new List<Int32>() { 3, 5, 6, 9, 10, 11 });

            // Communication Setup
            connection = new SerialConnection(this);

            FirmataProtocol _protocol = new FirmataProtocol(this);
            _protocol.I2C_Setting(FirmataProtocol.ANALOG_PORT, 4, 5);
            protocol = _protocol;
        }
    }
    class MetasArduinoSerialDevice : Device
    {
        public static String Description = "Metas Arduino";

        public MetasArduinoSerialDevice(String _identifier)
            : base(_identifier)
        {
            // Small Board:
            //   IN:  A0
            //   OUT: D9, D13
            // Large Board:
            //   IN:  A0, A1, A2, D2
            //   OUT: D5, D8, D9, D10

            /*
            PinResources.Add(PinType.INPUT,  new List<Int32>() { 2 });
            PinResources.Add(PinType.OUTPUT, new List<Int32>() { 5, 8, 9, 10, 13 });
            PinResources.Add(PinType.ANALOG, new List<Int32>() { 0, 1, 2 });
            PinResources.Add(PinType.PWM,    new List<Int32>() { 5, 9, 10 });
            */

            // Device Config
            PinResources.Add(PinType.INPUT, new List<Int32>() { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 });      //Pin 13 is not recommended to use as INPUT
            PinResources.Add(PinType.OUTPUT, new List<Int32>() { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 });  //Pin 0 and Pin 1 cannot be used (UART)
            PinResources.Add(PinType.ANALOG, new List<Int32>() { 0, 1, 2, 3, 4, 5 });
            PinResources.Add(PinType.PWM, new List<Int32>() { 3, 5, 6, 9, 10, 11 });
            PinResources.Add(PinType.SERVO, new List<Int32>() { 3, 5, 6, 9, 10, 11 });

            // Communication Setup
            connection = new SerialConnection(this);

            FirmataProtocol _protocol = new FirmataProtocol(this);
            _protocol.I2C_Setting(FirmataProtocol.ANALOG_PORT, 4, 5);
            protocol = _protocol;
        }
    }
    class MetasNodeMCUDevice : Device
    {
        public static String Description = "Metas X NodeMCU";
        
        public MetasNodeMCUDevice(String _identifier)
            : base(_identifier)
        {
            // Device Config
            PinResources.Add(PinType.OUTPUT, new List<Int32>() { 0, 1, 2, 3, 4, 5 ,8});
            PinResources.Add(PinType.ANALOG, new List<Int32>() { 0, 1 });
            PinResources.Add(PinType.INPUT, new List<Int32>() { 0, 3, 4, 5 });
            PinResources.Add(PinType.PWM, new List<Int32>() { 0, 1, 2, 3, 4, 5, 8 });
            PinResources.Add(PinType.SERVO, new List<Int32>() { 1, 2, 3, 4 });

            FunctionResources.Add("motor-forward");
            FunctionResources.Add("motor-backward");
            FunctionResources.Add("motor-left");
            FunctionResources.Add("motor-right");
            FunctionResources.Add("motor-stop");

            // Communication Setup
            connection = new HTTPConnection(this);
            protocol = new ARestSimpleProtocol(this);
        }
    }
    class MetasNodeMCUIPDevice : Device
    {
        public static String Description = "Metas X NodeMCU (IP Version)";

        public MetasNodeMCUIPDevice(String _identifier)
            : base(_identifier)
        {
            // Device Config
            PinResources.Add(PinType.OUTPUT, new List<Int32>() { 0, 1, 2, 3, 4, 5, 8 });
            PinResources.Add(PinType.ANALOG, new List<Int32>() { 0, 1 });
            PinResources.Add(PinType.INPUT, new List<Int32>() { 0, 3, 4, 5 });
            PinResources.Add(PinType.PWM, new List<Int32>() { 0, 1, 2, 3, 4, 5, 8 });
            PinResources.Add(PinType.SERVO, new List<Int32>() { 1, 2, 3, 4 });

            FunctionResources.Add("motor-forward");
            FunctionResources.Add("motor-backward");
            FunctionResources.Add("motor-left");
            FunctionResources.Add("motor-right");
            FunctionResources.Add("motor-stop");

            // Communication Setup
            connection = new HTTPConnection(this);
            protocol = new ARestIPProtocol(this);
        }
    }
    class IOTBoxDevice : Device
    {
        public static String Description = "IOT Box";

        public IOTBoxDevice(String _identifier)
            : base(_identifier)
        {
            // Device Config
            PinResources.Add(PinType.INPUT, new List<Int32>() { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 });       //Pin 13 is not recommended to use as INPUT
            PinResources.Add(PinType.OUTPUT, new List<Int32>() { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 });  //Pin 0 and Pin 1 cannot be used (UART)
            PinResources.Add(PinType.ANALOG, new List<Int32>() { 0, 1, 2, 3, 4, 5 });
            PinResources.Add(PinType.PWM, new List<Int32>() { 3, 5, 6, 9, 10, 11 });
            PinResources.Add(PinType.SERVO, new List<Int32>() { 3, 5, 6, 9, 10, 11 });

            // Communication Setup
            connection = new UDPConnection(this);

            FirmataProtocol _protocol = new FirmataProtocol(this);
            _protocol.I2C_Setting(FirmataProtocol.ANALOG_PORT, 4, 5);
            protocol = _protocol;

        }
    }

}
