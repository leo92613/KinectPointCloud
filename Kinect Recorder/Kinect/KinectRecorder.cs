/**
The MIT License (MIT)

Copyright (c) 2014 Lars Ivar Hatledal

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
**/


// Modified Kinect sample code, which enabled the recording of colored 3D point clouds
// Recordings are saved as zipped .ply files, where the whole .zip file correspond to a 'movie clip'.
// Single .ply files may be extracted and visualized in a software like MeshLab
//
// http://laht.info

namespace laht.info
{
    using System;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.IO.Compression;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Reflection;
    using System.Text;
    using System.Threading;
    using System.Collections;


    public class KinectRecorder
    {

        private string path = @"cloud" + DateTime.SpecifyKind(DateTime.Now, DateTimeKind.Local).ToShortDateString() + ".zip";

        private const int minNumOfPoints = 10000;
        private const int maxNumOfFrames = 2000;
        private const int step = 2;

        private bool done = false;
        private int count = 0;

        private ArrayList clouds = new ArrayList();
        private Stopwatch t0 = new Stopwatch();


        /// <summary>
        /// Indicates opaque in an opacity mask
        /// </summary>
        private const int OpaquePixel = -1;

        /// <summary>
        /// Size of the RGB pixel in the bitmap
        /// </summary>
        private readonly int bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for depth/color/body index frames
        /// </summary>
        private MultiSourceFrameReader multiFrameSourceReader = null;

        /// <summary>
        /// Intermediate storage for receiving depth frame data from the sensor
        /// </summary>
        private ushort[] depthFrameData = null;

        /// <summary>
        /// Intermediate storage for receiving color frame data from the sensor
        /// </summary>
        private byte[] colorFrameData = null;

        /// <summary>
        /// Intermediate storage for receiving body index frame data from the sensor
        /// </summary>
        private byte[] bodyIndexFrameData = null;

        /// <summary>
        /// Intermediate storage for the depth to color mapping
        /// </summary>
        private ColorSpacePoint[] colorPoints = null;

        /// <summary>
        /// Intermediate storage for the depth to color mapping
        /// </summary>
        private CameraSpacePoint[] cameraPoints = null;


        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public KinectRecorder()
        {

            //start clock
            this.t0.Start();

            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // open multiframereader for depth, color, and bodyindex frames
            this.multiFrameSourceReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color | FrameSourceTypes.BodyIndex);

            // wire handler for frames arrival
            this.multiFrameSourceReader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get FrameDescription from DepthFrameSource
            FrameDescription depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            int depthWidth = depthFrameDescription.Width;
            int depthHeight = depthFrameDescription.Height;

            // allocate space to put the pixels being received and converted
            this.depthFrameData = new ushort[depthWidth * depthHeight];
            this.bodyIndexFrameData = new byte[depthWidth * depthHeight];
            this.colorPoints = new ColorSpacePoint[depthWidth * depthHeight];
            this.cameraPoints = new CameraSpacePoint[depthWidth * depthHeight];

            // get FrameDescription from ColorFrameSource
            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.FrameDescription;

            int colorWidth = colorFrameDescription.Width;
            int colorHeight = colorFrameDescription.Height;

            // allocate space to put the pixels being received
            this.colorFrameData = new byte[colorWidth * colorHeight * this.bytesPerPixel];

            // open the sensor
            this.kinectSensor.Open();

            Console.WriteLine("Press any key to exit..");
            Console.ReadLine();

            Console.WriteLine("Flushing data to disk..");
            Console.WriteLine(this.path);
            using (FileStream zipToOpen = new FileStream(this.path, FileMode.Create))
            {
                using (ZipArchive zipArchive = new ZipArchive(zipToOpen, ZipArchiveMode.Update))
                {
                    for (int i = 0; i < clouds.Count; i++)
                    {
                        ZipArchiveEntry zipEntry = zipArchive.CreateEntry("cloud" + i + ".off");
                        using (StreamWriter writer = new StreamWriter(zipEntry.Open()))
                        {
                            writer.WriteLine(clouds[i]);
                        }
                        Console.WriteLine("Flushing " + i + " of " + clouds.Count);
                    }
                    Console.WriteLine("Shuting down..");
                }
            }


            this.multiFrameSourceReader.Dispose();
            this.multiFrameSourceReader = null;

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }



        /// <summary>
        /// Handles the depth/color/body index frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            if (!done)
            {
                if (t0.ElapsedMilliseconds > 41)
                {
                     //Console.WriteLine(t0.ElapsedMilliseconds);
                    t0.Restart();

                    int depthWidth = 0;
                    int depthHeight = 0;

                    int colorWidth = 0;
                    int colorHeight = 0;

                    int bodyIndexWidth = 0;
                    int bodyIndexHeight = 0;

                    bool multiSourceFrameProcessed = false;
                    bool colorFrameProcessed = false;
                    bool depthFrameProcessed = false;
                    bool bodyIndexFrameProcessed = false;

                    MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

                    if (multiSourceFrame != null)
                    {
                        // Frame Acquisition should always occur first when using multiSourceFrameReader
                        using (DepthFrame depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame())
                        {
                            using (ColorFrame colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame())
                            {
                                using (BodyIndexFrame bodyIndexFrame = multiSourceFrame.BodyIndexFrameReference.AcquireFrame())
                                {
                                    if (depthFrame != null)
                                    {
                                        FrameDescription depthFrameDescription = depthFrame.FrameDescription;
                                        depthWidth = depthFrameDescription.Width;
                                        depthHeight = depthFrameDescription.Height;

                                        if ((depthWidth * depthHeight) == this.depthFrameData.Length)
                                        {
                                            depthFrame.CopyFrameDataToArray(this.depthFrameData);

                                            depthFrameProcessed = true;
                                        }
                                    }

                                    if (colorFrame != null)
                                    {
                                        FrameDescription colorFrameDescription = colorFrame.FrameDescription;
                                        colorWidth = colorFrameDescription.Width;
                                        colorHeight = colorFrameDescription.Height;

                                        if ((colorWidth * colorHeight * this.bytesPerPixel) == this.colorFrameData.Length)
                                        {
                                            if (colorFrame.RawColorImageFormat == ColorImageFormat.Bgra)
                                            {
                                                colorFrame.CopyRawFrameDataToArray(this.colorFrameData);
                                            }
                                            else
                                            {
                                                colorFrame.CopyConvertedFrameDataToArray(this.colorFrameData, ColorImageFormat.Bgra);
                                            }

                                            colorFrameProcessed = true;
                                        }
                                    }

                                    if (bodyIndexFrame != null)
                                    {
                                        FrameDescription bodyIndexFrameDescription = bodyIndexFrame.FrameDescription;
                                        bodyIndexWidth = bodyIndexFrameDescription.Width;
                                        bodyIndexHeight = bodyIndexFrameDescription.Height;

                                        if ((bodyIndexWidth * bodyIndexHeight) == this.bodyIndexFrameData.Length)
                                        {
                                            bodyIndexFrame.CopyFrameDataToArray(this.bodyIndexFrameData);

                                            bodyIndexFrameProcessed = true;
                                        }
                                    }
                                    multiSourceFrameProcessed = true;
                                }
                            }
                        }
                    }



                    // we got all frames
                    if (multiSourceFrameProcessed && depthFrameProcessed && colorFrameProcessed && bodyIndexFrameProcessed)
                    {
                        StringBuilder sb = new StringBuilder();
                        int len = 0;

                        this.coordinateMapper.MapDepthFrameToColorSpace(this.depthFrameData, this.colorPoints);
                        this.coordinateMapper.MapDepthFrameToCameraSpace(this.depthFrameData, this.cameraPoints);

                        // loop over each row and column of the depth
                        for (int y = 0; y < depthHeight; y += step)
                        {
                            for (int x = 0; x < depthWidth; x += step)
                            {

                                // calculate index into depth array
                                int depthIndex = (y * depthWidth) + x;

                                byte player = this.bodyIndexFrameData[depthIndex];

                                // if we're tracking a player for the current pixel, sets its color and alpha to full
                                if (player != 0xff)
                                {

                                    CameraSpacePoint p = this.cameraPoints[depthIndex];

                                    // retrieve the depth to color mapping for the current depth pixel
                                    ColorSpacePoint colorPoint = this.colorPoints[depthIndex];

                                    byte r = 0; byte g = 0; byte b = 0;

                                    // make sure the depth pixel maps to a valid point in color space
                                    int colorX = (int)Math.Floor(colorPoint.X + 0.5);
                                    int colorY = (int)Math.Floor(colorPoint.Y + 0.5);
                                    if ((colorX >= 0) && (colorX < colorWidth) && (colorY >= 0) && (colorY < colorHeight))
                                    {
                                        // calculate index into color array
                                        int colorIndex = ((colorY * colorWidth) + colorX) * this.bytesPerPixel;

                                        // set source for copy to the color pixel
                                        int displayIndex = depthIndex * this.bytesPerPixel;

                                        b = this.colorFrameData[colorIndex++];
                                        g = this.colorFrameData[colorIndex++];
                                        r = this.colorFrameData[colorIndex++];

                                    }

                                    if (!(Double.IsInfinity(p.X)) && !(Double.IsInfinity(p.Y)) && !(Double.IsInfinity(p.Z)))
                                    {
                                        sb.Append(String.Format(CultureInfo.InvariantCulture, "{0} {1} {2} {3} {4} {5}\n", p.X, p.Y, p.Z, r, g, b,255));
                                        len++;
                                    }
                                }
                            }
                        }

                        if (len > minNumOfPoints && count < maxNumOfFrames)
                        {
                            //String header = "ply \n" +
                            //            "format ascii 1.0 \n" +
                            //            "element vertex " + len + "\n" +
                            //            "property float x \n" +
                            //            "property float y \n" +
                            //            "property float z \n" +
                            //            "property uchar red \n" +
                            //            "property uchar green \n" +
                            //            "property uchar blue \n" +
                            //            "end_header \n";
                            String header = "COFF\n" +
                                            len + " " + len * 4 + "\n";
                            sb.Insert(0, header);

                            WriteFile(count, sb);
                            //clouds.Add(sb.ToString());
                            Console.WriteLine("Wrote cloud " + count);
                            count++;
                        }
                        else if (count == maxNumOfFrames)
                        {
                            Console.WriteLine("Done.. Press any key to exit");
                            done = true;
                        }
                    }
                }
            }
        }

        public static void Main()
        {
            Thread.Sleep(2000);
            new KinectRecorder();
        }

        public static void WriteFile(int c, StringBuilder s)
        {
            string pp = @"cloud\" + c + ".off";
            FileStream FileToOpen = File.Create(pp);
            StreamWriter writer = new StreamWriter(FileToOpen);
            writer.WriteLine(s.ToString());
            writer.Close();
;            FileToOpen.Close();
        }
    }
}
