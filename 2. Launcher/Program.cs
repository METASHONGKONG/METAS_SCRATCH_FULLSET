using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using System.Windows.Forms;

namespace MakkoLocalServer
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            // Make sure no multiple instances
            bool result;
            var mutex = new System.Threading.Mutex(true, "MetasScratch", out result);
            if (!result) return;

            // Prepare Loading Screen
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            LoadingScreen loadingscreen = new LoadingScreen();
            Object loadingscreenLocker = new Object();
            ManualResetEvent loadingscreensync = new ManualResetEvent(false);

            // Starting Server in Seperate Thread
            Boolean Success = true;
            new Thread(delegate() {
                try {
                    MainSystem.Start();
                } catch(Exception) {
                    Success = false;
                }
                Thread.Sleep(1500);
                loadingscreensync.WaitOne();
                loadingscreen.Invoke((MethodInvoker)delegate() { lock (loadingscreenLocker) { loadingscreen.Close(); } });
            }).Start();
            

            // Show Loading Screen
            lock (loadingscreenLocker) {
                loadingscreensync.Set();
                if (!loadingscreen.IsDisposed) Application.Run(loadingscreen);
            }

            // Check Success / Failure
            if (!Success) {
                MessageBox.Show("Error occurs during initialization. Please restart the program to retry.");
                return;
            }

            // Start Flash
            Process flashInterface = new Process();
            try
            {
                flashInterface.StartInfo.FileName = "MetasScratch.exe";
                flashInterface.StartInfo.UseShellExecute = false;
                flashInterface.EnableRaisingEvents = true;
                flashInterface.Exited += flashInterface_Exited;
                flashInterface.Start();
            } catch (Exception) {
                MessageBox.Show("MetasScratch.exe is not found. Please reinstall the application.");
                Exit();
            }
            GC.KeepAlive(mutex); 
        }


        static void flashInterface_Exited(object sender, EventArgs e)
        {
            Exit();
        }
        static void Exit()
        {
            MainSystem.Stop();
        }
    }
}
