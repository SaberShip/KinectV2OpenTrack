using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace KinectV2OpenTrack
{
    class Program
    {
        static void Main(string[] args)
        {
            HeadTracker tracker = new HeadTracker();
            tracker.InitTracker();
            tracker.StartTracking();
        }
    }
}
