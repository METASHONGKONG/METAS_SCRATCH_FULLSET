using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using System.Threading;

namespace MakkoLocalServer
{
    enum Error
    {
        Unknown = 0,


        // Input Checking (Format Validity)
        Input_Identifier_Invalid = 101,
        Input_NoPacketIndex = 102,
        Input_NotNumeric = 103,
        Input_UnkownPacket = 104,

        // Input Checking (Range or Set Checking)
        Input_PinType_Mismatch = 111,
        Input_FunctionName_Mismatch = 112,
        Input_I2C_NotSupported = 113,

        // Input Checking (Search or Process)
        Input_Identifier_NotFound = 121, // No Connected Device with the input DeviceID found


        // Connection Layer
        TCP_Send_Failed = 201, // TCP Connection Send returns unsuccess
        Serial_Send_Failed = 221, // Serial Write throw Exception

        // Protocol Layer
        Firmata_Invalid_Version = 301,
        Firmata_Old_Version = 302,
        Firmata_I2C_Error = 303,
        Firmata_Read_NoRespond = 304,
        aRest_DHT_Error = 311,
        aRest_ConnectionLost = 312,
        aRest_ADCError = 313,
        aRest_I2C_Read_NumOfByteMismatch = 314,
        
        // 4xx reserved for flash

        // Device Layer
        Device_Restarted = 501,

        // .NET System Level
        System_SubNetNotFound = 601,
        System_Aborted = 602,

        // Compile Time
        Compile_Time_Mistake_PinTypeUnsupported = 901,
        Compile_Time_Mistake_Unimplemented = 902,
        Compile_Time_Mistake_FunctionUnimplemented = 903
    }

    class ClassifiedException : Exception
    {
        public Int32 ErrorNum = 0;

        public ClassifiedException(Error _error)
        {
            ErrorNum = (Int32) _error;
        }
        public ClassifiedException(Exception _ex)
        {
            if (_ex is ClassifiedException)
                ErrorNum = ((ClassifiedException)_ex).ErrorNum;
            else if (_ex is ThreadAbortException)
                ErrorNum = (Int32)Error.System_Aborted;
            else
                ErrorNum = 0;
        }
    }
}
