using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Net;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.Kinect;
using Microsoft.Kinect.Face;

namespace KinectV2OpenTrack
{
    class HeadTracker
    {
        private KinectSensor sensor = null;
        private BodyFrameSource bodySource = null;
        private BodyFrameReader bodyReader = null;
        private HighDefinitionFaceFrameSource hdFaceFrameSource = null;
        private HighDefinitionFaceFrameReader hdFaceFrameReader = null;

        private FaceAlignment faceAlignment = null;
        private Body trackedBody = null;
        private ulong currentTrackingId = 0;

        private HighDefinitionFaceFrame faceFrame = null;
        private bool lastSensorAvail = false;
        private bool tracking = false;


        private const int port = 4242;
        private const string ip = "127.0.0.1";
        private Socket socket;
        private IPAddress dest;
        private IPEndPoint endPoint;
        private byte[] sendBuffer;

        public void InitTracker()
        {
            lastSensorAvail = false;
            sensor = KinectSensor.GetDefault();

            bodySource = sensor.BodyFrameSource;
            bodyReader = bodySource.OpenReader();
            bodyReader.FrameArrived += NewBodyReaderFrame;

            hdFaceFrameSource = new HighDefinitionFaceFrameSource(sensor);
            hdFaceFrameSource.TrackingIdLost += HdFaceSource_TrackingIdLost;
            
            hdFaceFrameReader = hdFaceFrameSource.OpenReader();
            hdFaceFrameReader.FrameArrived += HdFaceReader_FrameArrived;
            
            sensor.IsAvailableChanged += SensorAvailableChanged;
            Console.WriteLine("Face tracker ready.");
            
            dest = IPAddress.Parse(ip);
            endPoint = new IPEndPoint(dest, port);
            
            sendBuffer = new byte[48];

            Console.WriteLine("UDP Socket created for port {0}", port);
        }


        private void AwaitSensor()
        {
            Console.WriteLine("Waiting for Kinect Sensor...");
            sensor.Open();
            
            tracking = false;
            while(!tracking)
            {
                Thread.Sleep(50);
            }

            StartTracking();
        }


        public void StartTracking()
        {

            if(!sensor.IsAvailable)
            {
                this.AwaitSensor();
            }

            
            faceAlignment = new FaceAlignment();

            Console.WriteLine("Started Face Tracking.");
            this.Run();
        }


        private void Run()
        {
            List<byte> bytes = new List<byte>();

            tracking = true;
            while (tracking)
            {
                if (faceFrame == null || !faceFrame.IsFaceTracked)
                {
                    Thread.Sleep(100);
                    continue;
                }

                // Orientation Quaternion
                double qW = faceAlignment.FaceOrientation.W;
                double qX = faceAlignment.FaceOrientation.X;
                double qY = faceAlignment.FaceOrientation.Y;
                double qZ = faceAlignment.FaceOrientation.Z;

                // Location in 3D space in cm
                double lX = 100 * faceAlignment.HeadPivotPoint.X;
                double lY = 100 * faceAlignment.HeadPivotPoint.Y;
                double lZ = 100 * faceAlignment.HeadPivotPoint.Z;
                
                // Get Euler angles in radians
                double yaw = Math.Asin(2.0 * (qX * qZ - qW * qY));
                double pitch = Math.Atan2(2.0 * (qW * qX + qY * qZ), 1.0 - 2.0 * (qX * qX + qY * qY));
                double roll = Math.Atan2(2.0 * qX * qY + 2.0 * qZ * qW, 1.0 - 2.0 * (qY * qY + qZ * qZ));


                bytes.Clear();
                bytes.AddRange(BitConverter.GetBytes(lX));
                bytes.AddRange(BitConverter.GetBytes(lY));
                bytes.AddRange(BitConverter.GetBytes(lZ));
                bytes.AddRange(BitConverter.GetBytes(180 / Math.PI * yaw));
                bytes.AddRange(BitConverter.GetBytes(180 / Math.PI * pitch));
                bytes.AddRange(BitConverter.GetBytes(180 / Math.PI * roll));

                sendBuffer = bytes.ToArray();

                try
                {
                    using (socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp))
                    {
                        socket.SendTo(sendBuffer, endPoint);
                    }
                }
                catch (Exception send_exception)
                {
                    Console.WriteLine("UDP Socket Exception: {0}", send_exception.Message);
                }

                //Console.WriteLine("X: {0}  Y: {1}  Z: {2}", lX, lY, lZ);
                //Console.WriteLine("Yaw: {0}  Pitch: {1}  Roll: {2}", yaw, pitch, roll);

                Thread.Sleep(25);
            }

            this.AwaitSensor();
        }


        private void NewBodyReaderFrame(object sender, BodyFrameArrivedEventArgs e)
        {
            var frameReference = e.FrameReference;
            using (var frame = frameReference.AcquireFrame())
            {
                if (frame == null)
                {
                    return;
                }

                if (this.trackedBody != null)
                {
                    this.trackedBody = FindBodyWithTrackingId(frame, this.currentTrackingId);

                    if (this.trackedBody != null)
                    {
                        return;
                    }
                }

                Body selectedBody = FindClosestBody(frame);

                if (selectedBody == null)
                {
                    return;
                }

                this.trackedBody = selectedBody;
                this.currentTrackingId = selectedBody.TrackingId;

                this.hdFaceFrameSource.TrackingId = this.currentTrackingId;

                Console.WriteLine("Tracking new face: " + hdFaceFrameSource.TrackingId.ToString());
            }
        }


        private void SensorAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            if (lastSensorAvail != e.IsAvailable)
            {
                if (!e.IsAvailable)
                {
                    Console.WriteLine("Kinect Sensor Disconnected.");
                    lastSensorAvail = false;
                    tracking = false;
                }
                else
                {
                    Console.WriteLine("Kinect Sensor Detected.");
                    lastSensorAvail = true;
                    tracking = true;
                }
            }
        }

        
        private void HdFaceSource_TrackingIdLost(object sender, TrackingIdLostEventArgs e)
        {
            var lostTrackingID = e.TrackingId;
            Console.WriteLine("Lost track of face: " + lostTrackingID.ToString());

            if (this.currentTrackingId == lostTrackingID)
            {
                this.currentTrackingId = 0;
                this.trackedBody = null;

                this.hdFaceFrameSource.TrackingId = 0;
            }
        }

        
        private void HdFaceReader_FrameArrived(object sender, HighDefinitionFaceFrameArrivedEventArgs e)
        {
            faceFrame = e.FrameReference.AcquireFrame();
            if (faceFrame == null || !faceFrame.IsFaceTracked)
            {
                return;
            }

            faceFrame.GetAndRefreshFaceAlignmentResult(this.faceAlignment);
        }

        
        private static double VectorLength(CameraSpacePoint point)
        {
            var result = Math.Pow(point.X, 2) + Math.Pow(point.Y, 2) + Math.Pow(point.Z, 2);

            result = Math.Sqrt(result);

            return result;
        }

        private static Body FindClosestBody(BodyFrame bodyFrame)
        {
            Body result = null;
            double closestBodyDistance = double.MaxValue;

            Body[] bodies = new Body[bodyFrame.BodyCount];
            bodyFrame.GetAndRefreshBodyData(bodies);

            foreach (var body in bodies)
            {
                if (body.IsTracked)
                {
                    var currentLocation = body.Joints[JointType.SpineBase].Position;

                    var currentDistance = VectorLength(currentLocation);

                    if (result == null || currentDistance < closestBodyDistance)
                    {
                        result = body;
                        closestBodyDistance = currentDistance;
                    }
                }
            }

            return result;
        }

        private static Body FindBodyWithTrackingId(BodyFrame bodyFrame, ulong trackingId)
        {
            Body result = null;

            Body[] bodies = new Body[bodyFrame.BodyCount];
            bodyFrame.GetAndRefreshBodyData(bodies);

            foreach (var body in bodies)
            {
                if (body.IsTracked)
                {
                    if (body.TrackingId == trackingId)
                    {
                        result = body;
                        break;
                    }
                }
            }

            return result;
        }
    }
}
