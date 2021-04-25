using CommandLine;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace KinectV2OpenTrack
{
    class Program
    {
        public class Options
        {
            [Option('i', "ip", Required = false, Default = "127.0.0.1", HelpText = "Ip address of the computer running OpenTrack")]
            public string Ip { get; set; }

            [Option('p', "port", Required = false, Default = 4242, HelpText = "Port the OpenTrack instance is listening on")]
            public int Port { get; set; }
        }

        static void Main(string[] args)
        {
            Parser.Default.ParseArguments<Options>(args)
                .WithParsed<Options>(o =>
                {
                    HeadTracker tracker = new HeadTracker();
                    tracker.InitTracker(o);
                    tracker.StartTracking();
                });
        }
    }
}
