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
using static KinectV2OpenTrack.Program;

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

        private bool lastSensorAvail = false;
        private bool tracking = false;

        private Socket socket;
        private IPAddress dest;
        private IPEndPoint endPoint;
        private byte[] sendBuffer;

        // Snapshot of the last good pose (x, y, z, yaw, pitch, roll) guarded by poseLock.
        private readonly object poseLock = new object();
        private readonly double[] lastPose = new double[6];
        private DateTime lastPoseTime;
        private bool havePose = false;
        private bool lostReported = false;

        // How long we keep sending the last known pose while face frames drop out.
        private static readonly TimeSpan PoseGracePeriod = TimeSpan.FromMilliseconds(500);
        // Interval used to re-send the held pose during a brief dropout.
        private static readonly TimeSpan HoldResendInterval = TimeSpan.FromMilliseconds(33);

        private System.Timers.Timer holdTimer;
        private DateTime lastHoldSend;

        public void InitTracker(Options o)
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
            
            dest = IPAddress.Parse(o.Ip);
            endPoint = new IPEndPoint(dest, o.Port);

            sendBuffer = new byte[48];

            // Create the UDP socket once and reuse it for every packet.
            // (Re-creating a socket per send adds unnecessary latency/overhead.)
            socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);

            // During brief face dropouts we keep re-sending the last known pose
            // so opentrack does not freeze on a stale value.
            holdTimer = new System.Timers.Timer(HoldResendInterval.TotalMilliseconds);
            holdTimer.Elapsed += HoldTimer_Elapsed;
            holdTimer.Start();

            Console.WriteLine("UDP Socket created for host {0}, port {1}", o.Ip, o.Port);
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
            if (!sensor.IsAvailable)
            {
                this.AwaitSensor();
            }
            else
            {
                sensor.Open();
            }

            faceAlignment = new FaceAlignment();

            Console.WriteLine("Started Face Tracking.");

            // Tracking output is now event-driven: every HD face frame that arrives is
            // converted and sent to opentrack immediately in HdFaceReader_FrameArrived.
            // Keep the main thread alive here.
            while (true)
            {
                Thread.Sleep(500);
            }
        }

        private void SendPose(double x, double y, double z, double yaw, double pitch, double roll)
        {
            // Opentrack UDP packet: 6 doubles (position in cm, angles in degrees).
            BitConverter.GetBytes(x).CopyTo(sendBuffer, 0);
            BitConverter.GetBytes(y).CopyTo(sendBuffer, 8);
            BitConverter.GetBytes(z).CopyTo(sendBuffer, 16);
            BitConverter.GetBytes(yaw).CopyTo(sendBuffer, 24);
            BitConverter.GetBytes(pitch).CopyTo(sendBuffer, 32);
            BitConverter.GetBytes(roll).CopyTo(sendBuffer, 40);

            try
            {
                socket.SendTo(sendBuffer, endPoint);
            }
            catch (Exception send_exception)
            {
                Console.WriteLine("UDP Socket Exception: {0}", send_exception.Message);
            }
        }

        private void HoldTimer_Elapsed(object sender, System.Timers.ElapsedEventArgs e)
        {
            double[] pose;

            lock (poseLock)
            {
                if (!havePose)
                {
                    return;
                }

                if (DateTime.UtcNow - lastPoseTime > PoseGracePeriod)
                {
                    // Dropout lasted longer than the grace period: stop holding the pose.
                    havePose = false;
                    if (!lostReported)
                    {
                        lostReported = true;
                        Console.WriteLine("Face tracking signal lost.");
                    }
                    return;
                }

                if (DateTime.UtcNow - lastHoldSend < HoldResendInterval)
                {
                    return;
                }

                lastHoldSend = DateTime.UtcNow;
                pose = (double[])lastPose.Clone();
            }

            SendPose(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
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
            // Always dispose frames. Leaking native frames can stall the reader,
            // which was a likely cause of tracking dropping out.
            using (var frame = e.FrameReference.AcquireFrame())
            {
                if (frame == null || !frame.IsFaceTracked)
                {
                    return;
                }

                frame.GetAndRefreshFaceAlignmentResult(this.faceAlignment);

                // Orientation Quaternion
                double qW = this.faceAlignment.FaceOrientation.W;
                double qX = this.faceAlignment.FaceOrientation.X;
                double qY = this.faceAlignment.FaceOrientation.Y;
                double qZ = this.faceAlignment.FaceOrientation.Z;

                // Location in 3D space in cm
                double lX = 100 * this.faceAlignment.HeadPivotPoint.X;
                double lY = 100 * this.faceAlignment.HeadPivotPoint.Y;
                double lZ = 100 * this.faceAlignment.HeadPivotPoint.Z;

                // Get Euler angles in degrees
                double yaw = 180 / Math.PI * Math.Asin(2.0 * (qX * qZ - qW * qY));
                double pitch = 180 / Math.PI * Math.Atan2(2.0 * (qW * qX + qY * qZ), 1.0 - 2.0 * (qX * qX + qY * qY));
                double roll = 180 / Math.PI * Math.Atan2(2.0 * qX * qY + 2.0 * qZ * qW, 1.0 - 2.0 * (qY * qY + qZ * qZ));

                lock (poseLock)
                {
                    lastPose[0] = lX;
                    lastPose[1] = lY;
                    lastPose[2] = lZ;
                    lastPose[3] = yaw;
                    lastPose[4] = pitch;
                    lastPose[5] = roll;
                    lastPoseTime = DateTime.UtcNow;
                    havePose = true;
                    lostReported = false;
                }

                // Send immediately: this is the freshest possible pose, so latency
                // is limited only by the Kinect frame rate (~30 fps).
                SendPose(lX, lY, lZ, yaw, pitch, roll);
            }
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
            double bestScore = double.MaxValue;

            Body[] bodies = new Body[bodyFrame.BodyCount];
            bodyFrame.GetAndRefreshBodyData(bodies);

            foreach (var body in bodies)
            {
                if (body.IsTracked)
                {
                    // Prefer bodies whose head is already tracked: HD face tracking
                    // can only start on a body with a usable head, so a slightly
                    // closer body without a tracked head is a poor choice.
                    bool headTracked = body.Joints[JointType.Head].TrackingState
                                            == TrackingState.Tracked;

                    var headLocation = body.Joints[JointType.Head].Position;
                    double headDistance = VectorLength(headLocation);

                    // Bodies with a tracked head always win; among equals, the
                    // closest head wins. The +10.0 penalty pushes headless bodies
                    // behind any body with a tracked head at a realistic distance.
                    double score = headDistance + (headTracked ? 0.0 : 10.0);

                    if (score < bestScore)
                    {
                        result = body;
                        bestScore = score;
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
