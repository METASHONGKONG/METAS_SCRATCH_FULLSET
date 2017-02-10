using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;



using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Net.NetworkInformation;


namespace MakkoLocalServer
{
    static class FlashManager
    {

        static public void Start()
        {
            WorkerThread = new Thread(new ThreadStart(Worker));
            WorkerThread.Start();
        }
        static public void Stop()
        {
            if (WorkerThread == null) return;
            WorkerThread.Abort();
            WorkerThread.Join();
        }

        //Thread Safe Queuing System
        static public void AddResponse(ResponsePacket _packet)
        {
            lock (ResponseQueue)
            {
                ResponseQueue.Enqueue(_packet);
            }
            WorkerWriteAvailable.Set();
        }
        static volatile private Queue<ResponsePacket> ResponseQueue = new Queue<ResponsePacket>();

        static private Thread WorkerThread;
        static private void Worker() {
            //TcpListener server = new TcpListener(IPAddress.Parse("127.0.0.1"), 59049);
            TcpListener server = new TcpListener(IPAddress.Any, 59049);
            try
            {
                server.Start();

                while (true)
                {
                    // Wait until New Connection
                    while (!server.Pending()) Thread.Sleep(2000);

                    // Clean up
                    lock (ResponseQueue) ResponseQueue.Clear();

                    using (TcpClient connection = server.AcceptTcpClient())
                    using (NetworkStream networkStream = new NetworkStream(connection.Client))
                    {
                        //Congestion Control
                        connection.ReceiveBufferSize = 256;

                        //Split a Seperate Thread for Reading
                        WorkerReadingStart(networkStream);

                        //Split a Seperate Thread for Writing
                        WorkerWritingStart(networkStream);

                        //Wait until Disconnected or New Connection
                        TcpState state;
                        do
                        {
                            Thread.Sleep(5000);
                            TcpConnectionInformation tcpinfo = IPGlobalProperties.GetIPGlobalProperties().GetActiveTcpConnections().SingleOrDefault(x => (x.LocalEndPoint.Equals(connection.Client.LocalEndPoint) & x.RemoteEndPoint.Equals(connection.Client.RemoteEndPoint)));
                            state = tcpinfo != null ? tcpinfo.State : TcpState.Unknown;
                        } while (!server.Pending() && state != TcpState.Closed && state != TcpState.CloseWait && state != TcpState.Closing);

                        //Disconnect
                        WorkerWritingStop();
                        WorkerReadingStop();
                        networkStream.Close();
                        connection.Close();
                    }
                }
            }
            catch(Exception) //Including Thread Abort
            {
                WorkerWritingStop();
                WorkerReadingStop();
                server.Stop();
            }
        }

        static private AutoResetEvent WorkerWriteAvailable = new AutoResetEvent(false);
        static private void WorkerWritingStart(NetworkStream networkStream)
        {
            WorkerWritingThread = new Thread(new ParameterizedThreadStart(WorkerWriting));
            WorkerWritingThread.Start(networkStream);
        }
        static private void WorkerWritingStop()
        {
            if (WorkerWritingThread != null && WorkerWritingThread.IsAlive) WorkerWritingThread.Abort();
            if (WorkerWritingThread != null) WorkerWritingThread.Join();
        }
        static private Thread WorkerWritingThread;
        static private void WorkerWriting(Object parameter)
        {
            
            NetworkStream networkStream = (NetworkStream)parameter;
            while (true) {
                try {
                    //Check Queue, Wait until Having New Item
                    WorkerWriteAvailable.WaitOne();

                    //Pull From Queue until It is empty
                    Int32 remaining = 0;
                    do{
                        ResponsePacket packetBuffer;
                        lock (ResponseQueue)
                        {
                            if (ResponseQueue.Count <= 0) continue;
                            packetBuffer = ResponseQueue.Dequeue();
                        }
                        String xmlBuffer = packetBuffer.ToXML() + Char.MinValue;
                        Byte[] byteBuffer = System.Text.Encoding.ASCII.GetBytes(xmlBuffer);
                        if (networkStream.CanWrite) networkStream.Write(byteBuffer, 0, byteBuffer.Length);

                        lock (ResponseQueue) remaining = ResponseQueue.Count;
                    }
                    while (remaining > 0);
                } catch (Exception) { }
            }
        }

        static private Byte[] WorkerReadBuffer = new Byte[1024];
        static private String WorkerReadBufferString;
        static private void WorkerReadingStart(NetworkStream networkStream)
        {
            WorkerReadBuffer = new Byte[1024];
            WorkerReadBufferString = "";
            networkStream.BeginRead(WorkerReadBuffer, 0, WorkerReadBuffer.Length, WorkerReading, networkStream);
        }
        static private void WorkerReadingStop()
        {
            //networkStream.Close(); //Handled in Worker
            return; //Do Nothing
        }
        static private void WorkerReading(IAsyncResult iar)
        {
            try
            {
                NetworkStream networkStream = ((NetworkStream)iar.AsyncState);
                Int32 byteAvailable = networkStream.EndRead(iar);

                try {
                    for (Int32 i = 0; i < byteAvailable; i++ ) {
                        if (WorkerReadBuffer[i] == 0) {
                            String newPacketString = WorkerReadBufferString;
                            WorkerReadBufferString = ""; // Pre-cleaning

                            //Parse XML
                            RequestPacket newPacket = RequestPacket.Parse(newPacketString);
                            if (newPacket.Tag == "policy-file-request") {
                                //Reply with Cross-Domain-Policy
                                AddResponse(new CDPResponsePacket());
                            } else {
                                //Push to Device Manager if not Policy-File-Request
                                DeviceManager.PacketAnalyser(newPacket);
                            }
                        } else {
                            WorkerReadBufferString += (Char)WorkerReadBuffer[i];
                        }
                    }
                }
                catch (Exception) { /* Usually XML error */ }

                WorkerReadBuffer = new Byte[1024];
                networkStream.BeginRead(WorkerReadBuffer, 0, WorkerReadBuffer.Length, WorkerReading, networkStream);
            }
            catch (Exception) { /* Usually critial flash-loopback-network error */ }
        }


    }
}
