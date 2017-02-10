using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Xml;

namespace MakkoLocalServer
{
    // Base
    abstract class Packet
    {
        public String Tag
        {
            get;
            protected set;
        }
        public Int32? Index
        {
            get;
            protected set;
        }
        public List<KeyValuePair<String, String>> Args
        {
            get;
            protected set;
        }
    }
    class RequestPacket : Packet
    {
        static public RequestPacket Parse(String xml)
        {
            XmlDocument xmlDoc = new XmlDocument();
            xmlDoc.LoadXml(xml);

            RequestPacket returnPacket;

            // Extract Index
            XmlNodeList indexList = xmlDoc.DocumentElement.GetElementsByTagName("index");
            if (indexList.Count <= 0)
                returnPacket = new RequestPacket(xmlDoc.DocumentElement.Name);
            else
                returnPacket = new RequestPacket(xmlDoc.DocumentElement.Name, Int32.Parse(indexList.Item(0).InnerText));

            // Extract Args
            XmlNodeList argsList = xmlDoc.DocumentElement.GetElementsByTagName("arg");
            if (argsList.Count > 0)
            {
                foreach (XmlNode arg in argsList)
                {
                    returnPacket.Args.Add(new KeyValuePair<String, String>(arg.Name, arg.InnerText));
                }
            }

            return returnPacket;
        }

        public RequestPacket(String _tag)
        {
            Tag = _tag;
            Index = null;
            Args = new List<KeyValuePair<String, String>>();
        }
        public RequestPacket(String _tag, Int32 _index)
        {
            Tag = _tag;
            Index = _index;
            Args = new List<KeyValuePair<String, String>>();
        }
    }
    abstract class ResponsePacket :Packet
    {
        public ResponsePacket() { }

        public virtual String ToXML()
        {
            if (Tag == null || Tag == "") throw new Exception();

            String xml = "<" + Tag + ">";
            if (Index != null) xml += "<index>" + Index.ToString() + "</index>";
            if (Args != null && Args.Count > 0)
            {
                foreach(KeyValuePair<String, String> arg in Args)
                {
                    xml += "<" + arg.Key + ">" + arg.Value + "</" + arg.Key + ">";
                }
            }
            xml += "</" + Tag + ">";
            return xml;
        }
    }

    // Cross Domain Policy Response
    class CDPResponsePacket : ResponsePacket
    {
        public CDPResponsePacket() {
            Tag = "cross-domain-policy";
        }
        public override String ToXML()
        {
            return "<cross-domain-policy><allow-access-from domain=\"*\" to-ports=\"59049\"/></cross-domain-policy>";
        }
    }

    // Generic Success / Fail
    class GenericSuccessResponsePacket : ResponsePacket
    {
        public GenericSuccessResponsePacket(Int32 _index)
        {
            Tag = "succeed";
            Index = _index;
            Args = new List<KeyValuePair<String, String>>();
        }
    }
    class GenericFailResponsePacket : ResponsePacket
    {
        public GenericFailResponsePacket(Int32 _index)
        {
            Tag = "fail";
            Index = _index;
            Args = new List<KeyValuePair<String, String>>();
            Args.Add(new KeyValuePair<String, String>("error_no", "0"));
        }
        public GenericFailResponsePacket(Int32 _index, Exception _ex)
        {
            Tag = "fail";
            Index = _index;
            Args = new List<KeyValuePair<String, String>>();
            Args.Add(new KeyValuePair<String, String>("error_no", (new ClassifiedException(_ex)).ErrorNum.ToString()));
        }
    }

    // Special Success Response
    class DigitalReadSuccessResponsePacket : ResponsePacket
    {
        public DigitalReadSuccessResponsePacket(Int32 _index, Logic _value)
        {
            Tag = "succeed";
            Index = _index;
            Args = new List<KeyValuePair<String, String>>();
            Args.Add(new KeyValuePair<String, String>("value", LogicHelper.ToString(_value)));
        }
    }
    class AnalogReadSuccessResponsePacket : ResponsePacket
    {
        public AnalogReadSuccessResponsePacket(Int32 _index, Int32 _value)
        {
            Tag = "succeed";
            Index = _index;
            Args = new List<KeyValuePair<String, String>>();
            Args.Add(new KeyValuePair<String, String>("value", _value.ToString()));
        }
    }
    class I2CReadSuccessResponsePacket : ResponsePacket
    {
        public I2CReadSuccessResponsePacket(Int32 _index, List<Byte> _value)
        {
            Tag = "succeed";
            Index = _index;
            Args = new List<KeyValuePair<String, String>>();
            foreach (Byte _byte in _value) Args.Add(new KeyValuePair<String, String>("value", "0x" + _byte.ToString("X2")));
        }
    }
    class FunctionSuccessResponsePacket : ResponsePacket
    {
        public FunctionSuccessResponsePacket(Int32 _index, Int32 _value)
        {
            Tag = "succeed";
            Index = _index;
            Args = new List<KeyValuePair<String, String>>();
            Args.Add(new KeyValuePair<String, String>("value", _value.ToString()));
        }
    }
    class ScanDevicesResponsePacket : ResponsePacket
    {
        public ScanDevicesResponsePacket(Int32 _index)
        {
            Tag = "succeed";
            Index = _index;
            Args = new List<KeyValuePair<String, String>>();
        }
        public void AddDevice(String _identifier, String _description)
        {
            Args.Add(new KeyValuePair<String, String>("device", _identifier + "," + _description));
        }
    }

    // Callback / Anonymous Response
    class DevicesUpdateCallbackPacket : ResponsePacket
    {
        public DevicesUpdateCallbackPacket()
        {
            Tag = "devices";
            Index = null;
            Args = new List<KeyValuePair<String, String>>();
        }
        public void AddDevice(String _identifier, String _description)
        {
            Args.Add(new KeyValuePair<String, String>("device", _identifier + "," + _description));
        }
    }
    class FailCallbackPacket : ResponsePacket
    {
        public FailCallbackPacket(Exception _ex)
        {
            Tag = "fail";
            Index = null;
            Args = new List<KeyValuePair<String, String>>();
            Args.Add(new KeyValuePair<String, String>("error_no", (new ClassifiedException(_ex)).ErrorNum.ToString()));
        }
    }
    class DigitalReadCallbackPacket : ResponsePacket
    {
        public DigitalReadCallbackPacket(String _deviceID, Int32 _pin, Logic _value)
        {
            Tag = "digital-pin";
            Index = null;
            Args = new List<KeyValuePair<String, String>>();
            Args.Add(new KeyValuePair<String, String>("device_id", _deviceID));
            Args.Add(new KeyValuePair<String, String>("pin", _pin.ToString()));
            Args.Add(new KeyValuePair<String, String>("value", LogicHelper.ToString(_value)));
        }
    }
}
