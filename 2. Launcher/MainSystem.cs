using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MakkoLocalServer
{
    static class MainSystem
    {
        static public void Start()
        {
            FlashManager.Start();
            DeviceManager.Start();
            TCPIPDeviceEnumeration.Start();
            SerialDeviceEnumeration.Start();
        }

        static public void Stop()
        {
            try {
                SerialDeviceEnumeration.Stop();
                TCPIPDeviceEnumeration.Stop();
                DeviceManager.Stop();
                FlashManager.Stop();
            } catch(Exception) { }
        }
    }
}
