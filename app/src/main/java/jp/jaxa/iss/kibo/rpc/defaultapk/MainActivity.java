package jp.jaxa.iss.kibo.rpc.defaultapk;


import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.SurfaceView;
import android.widget.Toast;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Vector;

import static android.content.ContentValues.TAG;

/*
public class MainActivity extends AppCompatActivity implements CameraBridgeViewBase.CvCameraViewListener2 {
    //view holder
    CameraBridgeViewBase cameraBridgeViewBase;
    //camera listener callback
    BaseLoaderCallback baseLoaderCallback;
    //image holder
    Mat bwIMG, hsvIMG, lrrIMG, urrIMG, dsIMG, usIMG, cIMG, hovIMG;
    MatOfPoint2f approxCurve;
    int threshold;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        //initialize treshold
        threshold = 100;
        cameraBridgeViewBase = (JavaCameraView) findViewById(R.id.cameraViewer);
        cameraBridgeViewBase.setVisibility(SurfaceView.VISIBLE);
        cameraBridgeViewBase.setCvCameraViewListener(this);
        //create camera listener callback
        baseLoaderCallback = new BaseLoaderCallback(this) {
            @Override
            public void onManagerConnected(int status) {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS:
                        Log.v("aashari-log", "Loader interface success");
                        bwIMG = new Mat();
                        dsIMG = new Mat();
                        hsvIMG = new Mat();
                        lrrIMG = new Mat();
                        urrIMG = new Mat();
                        usIMG = new Mat();
                        cIMG = new Mat();
                        hovIMG = new Mat();
                        approxCurve = new MatOfPoint2f();
                        cameraBridgeViewBase.enableView();
                        break;
                    default:
                        super.onManagerConnected(status);
                        break;
                }
            }
        };
    }
    @Override
    public void onCameraViewStarted(int width, int height) {
    }
    @Override
    public void onCameraViewStopped() {
    }
    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
       // Dictionary dictionary =  Aruco.getPredefinedDictionary(Aruco.DICT_4X4_50);
       // Mat gimage = inputFrame.gray();
       // Mat rgb = inputFrame.rgba();
       // Mat ids = new Mat();
       // List<Mat> corners = new ArrayList<>();

       // Aruco.detectMarkers(gimage, dictionary, corners, ids);
       // Log.d("test",Integer.toString(corners.size()));

       // if (corners.size()>0)
       // {
       //     try {
       //         Log.d("test3",Arrays.toString(ids.get(0,0)));
       //         Log.d("test position : ",corners.get(0));
       //         //Imgproc.putText(rgb, "Aruco:", new Point(30, 80), 3, 0.5, new Scalar(255, 0, 0, 255), 1);
       //         //Aruco.drawDetectedMarkers(gimage, corners, rgb, new Scalar(0, 255, 0));
       //     }
       //     catch (Exception e){
       //         Log.d("test",e.getMessage());
       //     }
       // }
       // return rgb;
       //try {
       //     Mat src = findRectangle(inputFrame.gray());
       //     //Mat src = drawDetectedMarkers(inputFrame.gray());
       //     return src;
       // } catch (Exception e) {
       //     e.printStackTrace();
       //     return null;
       // }


       // Mat gray = inputFrame.gray();
       // Mat dst = inputFrame.rgba();
       // Imgproc.pyrDown(gray, dsIMG, new Size(gray.cols() / 2, gray.rows() / 2));
       // Imgproc.pyrUp(dsIMG, usIMG, gray.size());
       // Imgproc.Canny(usIMG, bwIMG, 0, threshold);
       // Imgproc.dilate(bwIMG, bwIMG, new Mat(), new Point(-1, 1), 1);
       // List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
       // cIMG = bwIMG.clone();
       // contours.clear();
       // Imgproc.findContours(cIMG, contours, hovIMG, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
       // for (MatOfPoint cnt : contours) {
       //     MatOfPoint2f curve = new MatOfPoint2f(cnt.toArray());
       //     Imgproc.approxPolyDP(curve, approxCurve, 0.02 * Imgproc.arcLength(curve, true), true);
       //     int numberVertices = (int) approxCurve.total();
       //     double contourArea = Imgproc.contourArea(cnt);
       //     if (Math.abs(contourArea) < 100) {
       //         continue;
       //     }
       //     //Rectangle detected
       //     if (numberVertices >= 4 && numberVertices <= 6 &&
       //             Math.abs(Imgproc.contourArea(new MatOfPoint(approxCurve.toArray()))) > 1000 &&
       //               Imgproc.isContourConvex(new MatOfPoint(approxCurve.toArray()))
       //     ) {
       //         List<Double> cos = new ArrayList<>();
       //         cos.clear();
       //         double maxCosine = 0;
       //         for (int j = 2; j < numberVertices + 1; j++) {
       //             cos.add(angle(approxCurve.toArray()[j % numberVertices], approxCurve.toArray()[j - 2], approxCurve.toArray()[j - 1]));
       //             //double cosine = Math.abs(angle(new MatOfPoint(approxCurve.toArray()).toArray()[j%4], new MatOfPoint(approxCurve.toArray()).toArray()[j-2], new MatOfPoint(approxCurve.toArray()).toArray()[j-1]));
       //             //maxCosine = Math.max(maxCosine, cosine);
       //         }
       //         Collections.sort(cos);
       //         double mincos = cos.get(0);
       //         double maxcos = cos.get(cos.size() - 1);
       //         if (numberVertices == 4 && mincos >= -0.1 && maxcos < 0.3) {
       //         //if (maxCosine < 0.3){
       //             //setLabel(dst, "X", cnt);
       //             setLabel(dst, "X", cnt,new Scalar(255,0,0));
       //             List<MatOfPoint> biggest = findBiggestRactangle(contours);
       //             if (biggest != null) {
       //                 setLabel(dst, "X", biggest.get(0), new Scalar(0, 255, 0));
       //                 maxRectanglePosition(biggest,dst.height(),dst.width());
       //             }
       //             //Imgproc.drawContours(dst, contours, -1, new Scalar(0,0,255));
       //             Imgproc.drawContours(dst,biggest,-1,new Scalar(255,0,0));
       //         }
       //     }
       // }
       // return dst;
    }
    private Mat  findRectangle(Mat src) throws Exception {
        Mat blurred = src.clone();
        Imgproc.medianBlur(src, blurred, 9);
        Mat gray0 = new Mat(blurred.size(), CvType.CV_8U);
        Mat gray = new Mat();
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        List<Mat> blurredChannel = new ArrayList<Mat>();
        blurredChannel.add(blurred);
        List<Mat> gray0Channel = new ArrayList<Mat>();
        gray0Channel.add(gray0);
        MatOfPoint2f approxCurve;
        double maxArea = 100;
        int maxId = -1;
        for (int c = 0; c < 3; c++) {
            int ch[] = { c, 0 };
            //Core.mixChannels(blurredChannel, gray0Channel, new MatOfInt(ch));
            Core.mixChannels(blurredChannel, gray0Channel, new MatOfInt(0,0));
            int thresholdLevel = 1;
            for (int t = 0; t < thresholdLevel; t++) {
                if (t == 0) {
                    Imgproc.Canny(gray0, gray, 10, 20, 3, true); // true ?
                    Imgproc.dilate(gray, gray, new Mat(), new Point(-1, -1), 1); // 1
                    // ?
                } else {
                    Imgproc.adaptiveThreshold(gray0, gray, thresholdLevel,
                            Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C,
                            Imgproc.THRESH_BINARY,
                            (src.width() + src.height()) / 200, t);
                }
                Imgproc.findContours(gray, contours, new Mat(),
                        Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
                for (MatOfPoint contour : contours) {
                    MatOfPoint2f temp = new MatOfPoint2f(contour.toArray());
                    double area = Imgproc.contourArea(contour);
                    approxCurve = new MatOfPoint2f();
                    Imgproc.approxPolyDP(temp, approxCurve,
                            Imgproc.arcLength(temp, true) * 0.02, true);
                    if (approxCurve.total() == 4 && area >= maxArea) {
                        double maxCosine = 0;
                        List<Point> curves = approxCurve.toList();
                        for (int j = 2; j < 5; j++) {
                            double cosine = Math.abs(angle(curves.get(j % 4),
                                    curves.get(j - 2), curves.get(j - 1)));
                            maxCosine = Math.max(maxCosine, cosine);
                        }
                        if (maxCosine < 0.3) {
                            maxArea = area;
                            maxId = contours.indexOf(contour);
                        }
                    }
                }
            }
        }
        if (maxId >= 0) {
            Imgproc.drawContours(src, contours, maxId, new Scalar(255, 0, 0,
                    .8), 8);
            maxRectanglePosition(contours.get(maxId),src.width(),src.height());
            //findBiggestRactangle()
        }
        return src;
    }
   // private void setLabel(Mat im, String label, MatOfPoint contour,Scalar color) {
   //     int fontface = Core.FONT_HERSHEY_SIMPLEX;
   //     double scale = 3;//0.4;
   //     int thickness = 3;//1;
   //     int[] baseline = new int[1];
   //     Size text = Imgproc.getTextSize(label, fontface, scale, thickness, baseline);
   //     Rect r = Imgproc.boundingRect(contour);
   //     Point pt = new Point(r.x + ((r.width - text.width) / 2),r.y + ((r.height + text.height) / 2));
   //     Imgproc.putText(im, label, pt, fontface, scale, color, thickness);
   // }
    List<MatOfPoint> squares = new ArrayList<MatOfPoint>();
   // public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
           // try {
           //     Mat source =  inputFrame.gray();
           //     Mat destination = new Mat(source.rows(), source.cols(), source.type());
           //     Mat edges = new Mat(source.rows(), source.cols(), source.type());
           //     Imgproc.cvtColor(source, destination, Imgproc.COLOR_RGB2GRAY);
           //     Imgproc.equalizeHist(destination, destination);
           //     Imgproc.GaussianBlur(destination, destination, new Size(5, 5), 0, 0, Core.BORDER_DEFAULT);
           //     int threshold = 100;
           //     Imgproc.Canny(destination, edges, threshold, threshold*3);
           //     List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
           //     Mat hierarchy = new Mat();
           //     Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
           //     MatOfPoint2f matOfPoint2f = new MatOfPoint2f();
           //     MatOfPoint2f approxCurve = new MatOfPoint2f();
           //     for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0]) {
           //         MatOfPoint contour = contours.get(idx);
           //         Rect rect = Imgproc.boundingRect(contour);
           //         double contourArea = Imgproc.contourArea(contour);
           //         matOfPoint2f.fromList(contour.toList());
           //         Imgproc.approxPolyDP(matOfPoint2f, approxCurve, Imgproc.arcLength(matOfPoint2f, true) * 0.02, true);
           //         long total = approxCurve.total();
           //         if (total == 3) { // is triangle
           //             // do things for triangle
           //             Log.d("Success","" + approxCurve.toArray()[0].x + " : " + approxCurve.toArray()[0].y);
           //         }
           //         if (total >= 4 && total <= 6) {
           //             List<Double> cos = new ArrayList<>();
           //             Point[] points = approxCurve.toArray();
           //             for (int j = 2; j < total + 1; j++) {
           //                 cos.add(angle(points[(int) (j % total)], points[j - 2], points[j - 1]));
           //             }
           //             Collections.sort(cos);
           //             Double minCos = cos.get(0);
           //             Double maxCos = cos.get(cos.size() - 1);
           //             boolean isRect = total == 4 && minCos >= -0.1 && maxCos <= 0.3;
           //             boolean isPolygon = (total == 5 && minCos >= -0.34 && maxCos <= -0.27) || (total == 6 && minCos >= -0.55 && maxCos <= -0.45);
           //             if (isRect) {
           //                 double ratio = Math.abs(1 - (double) rect.width / rect.height);
           //                 source = drawText(destination,rect.tl(), ratio <= 0.02 ? "SQU" : "RECT");
           //             }
           //             if (isPolygon) {
           //                 source = drawText(destination,rect.tl(), "Polygon");
           //             }
           //         }
           //     }
           //     return source;
                //Imgproc.Canny(destination, destination, 50, 100);
                //Imgproc.adaptiveThreshold(destination, destination, 255, Imgproc.ADAPTIVE_THRESH_MEAN_C, Imgproc.THRESH_BINARY, 15, 40);
               // Imgproc.threshold(destination, destination, 0, 255, Imgproc.THRESH_BINARY);
               // if (destination != null) {
               //     Mat lines = new Mat();
               //     Imgproc.HoughLinesP(destination, lines, 1, Math.PI / 180, 50, 30, 10);
               //     Mat houghLines = new Mat();
               //     houghLines.create(destination.rows(), destination.cols(), CvType.CV_8UC1);
               //     //Drawing lines on the image
               //     for (int i = 0; i < lines.cols(); i++) {
               //         double[] points = lines.get(0, i);
               //         double x1, y1, x2, y2;
               //         x1 = points[0];
               //         y1 = points[1];
               //         x2 = points[2];
               //         y2 = points[3];
               //         Point pt1 = new Point(x1, y1);
               //         Point pt2 = new Point(x2, y2);
               //         //Drawing lines on an image
               //         Imgproc.line(source, pt1, pt2, new Scalar(0, 0, 255), 4);
               //     }
               // }
               // //Imgcodecs.imwrite("rectangle_houghtransform.jpg", source);
               // return source;
           // } catch (Exception e) {
           //     System.out.println("error: " + e.getMessage());
           //     return inputFrame.gray();
           // }
        //rotate(inputFrame.rgba(),270);
        //rotate(inputFrame.gray(),270);
       // Mat mGray = inputFrame.gray();
       // if (Math.random()>0.80) {
       //     Imgproc.cvtColor(inputFrame.gray(),mGray,Imgproc.COLOR_GRAY2RGB);
       //     //findSquares(inputFrame.rgba().clone(),squares);
       //     findSquares(mGray.clone(),squares);
       // }
       // Mat image = inputFrame.rgba(); //960x720
       // Log.d("Image ",image.size().toString());
       // Log.d("Squares : ", String.valueOf(squares.size())); // 200
       // for (int i = 0; i < squares.size() ; i++) {
       //     Log.d("Mat Size x : ",Integer.toString(i) + " = " + squares.get(i).size().toString());
       //     Log.d("Mat x : ", squares.get(i).toArray()[0].toString()); // Left upper
       //     Log.d("Mat x1: ", squares.get(i).toArray()[1].toString()); // left lower
       //     Log.d("Mat x2: ", squares.get(i).toArray()[2].toString()); // right lower
       //     Log.d("Mat x3: ", squares.get(i).toArray()[3].toString()); // right upper
       // }
       // MatOfPoint approx = new MatOfPoint();
       // approx.fromArray(new Point(1,1),new Point(1,500),new Point(500,500),
       //         new Point(100,1));
       // List<MatOfPoint> squares2 = new ArrayList<MatOfPoint>();
       // squares2.add(approx);
       // maxRectanglePosition(findBiggestRactangle(squares),1280,720);
       // Imgproc.drawContours(image, squares, -1, new Scalar(0,0,255));
       // Imgproc.drawContours(image,findBiggestRactangle(squares),-1,new Scalar(255,0,0));
        //Imgproc.drawContours(image, squares2, -1, new Scalar(255,0,0));
        //image = rotationAffineTutorial(image);
        //Mat image = inputFrame.rgba();
        //return inputFrame.gray();
    //}
   // private Mat drawText(Mat colorImage,Point ofs, String text) {
   //     Imgproc.putText(colorImage, text, ofs, Core.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255,255,25));
   //     return colorImage;
   // }
    private void findSqaures(Mat sourceImage){
        Vector<Point> sqares;
        Mat pyr,timing ,gry =new Mat();
        pyr=new Mat(sourceImage.size(),CvType.CV_8U);
        timing=new Mat(sourceImage.size(),CvType.CV_8U);
        int thresh = 50, N = 11;
        List<Mat> grayO=new ArrayList<Mat>();
        List<Mat> timing1=new ArrayList<Mat>();
        Imgproc.pyrDown(sourceImage, pyr,new Size(sourceImage.cols()/2.0, sourceImage.rows()/2));
        Imgproc.pyrUp(pyr, timing,sourceImage.size());
//      Vector<Point> contours=new Vector<Point>();
        timing1.add(0,pyr);
        grayO.add(0,timing);
//      grayO.add(0,timing);
        for(int c=0;c<3;c++){
            int ch[]={1,0};
            MatOfInt fromto = new MatOfInt(ch);
            Core.mixChannels(timing1, grayO, fromto); // Getting Exception here
//          Core.mixChannels(src, dst, fromTo)
            for(int i=0;i<N;i++){
                Mat output=grayO.get(0);
                if(i==0){
                    Imgproc.Canny(output, gry, 5, thresh);
                    Imgproc.dilate(gry, gry, new Mat(), new Point(-1,-1), 1);
                }
                else {
//                   output = output >= (i+1)*255/N;
                }
//              sourceImage=gry;
                List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
                Imgproc.findContours(gry, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
                MatOfPoint2f approxCurve = new MatOfPoint2f();
               // mDrawnContours.clear();
               // Log.i(TAG, "::findSqaures:" + "contours.size():"+contours.size());
               // for(int j=0;i<contours.size();j++){
               //     MatOfPoint tempContour=contours.get(i);
               //     MatOfPoint2f newMat = new MatOfPoint2f( tempContour.toArray() );
               //     int contourSize = (int)tempContour.total();
               //     Imgproc.approxPolyDP(newMat, approxCurve, contourSize*0.02, true);
               //     MatOfPoint points=new MatOfPoint(approxCurve.toArray());
//             //       if( approx.size() == 4 && fabs(contourArea(cv::Mat(approx))) > 1000 && cv::isContourConvex(cv::Mat(approx))) {
               //     if(points.toArray().length==4 && (Math.abs(approxCurve.total())>1000) && Imgproc.isContourConvex(points)){
               //         double maxCosine=0;
               //         int k;
               //         for( k=2;k<5;k++){
               //             double cosine=Math.abs(angle(points.toArray()[k%4], points.toArray()[k-2], points.toArray()[k-1]));
               //             if(maxCosine>cosine){
               //                 maxCosine=cosine;
               //             }
               //         }
               //         Log.i(TAG, "::findSqaures:" + "maxCosine:"+maxCosine);
               //         if(maxCosine<0.3){
               //             DrawnContours drawnContours = new DrawnContours();
               //             drawnContours.setIndex(k);
               //             mDrawnContours.add(drawnContours);
               //         }
               //     }
               // }
               // Log.i(TAG, "::findSqaures:" + "mDrawnContours.size():"+mDrawnContours.size());
            }
        }
//      Core.mixChannels(src, dst, fromTo)
    }
    private void maxRectanglePosition(MatOfPoint listReg,int sizeX,int sizeY){
        String strPosition = "";
        if (listReg != null) {
            MatOfPoint maxRectangle = listReg;
            Log.d("Num", Double.toString(maxRectangle.toArray()[0].x)
                    + " : " + Double.toString(sizeX - maxRectangle.toArray()[2].x));
            if ((maxRectangle.toArray()[0].x) < (sizeX - maxRectangle.toArray()[2].x)) {
                strPosition = "in left";
            } else if ((maxRectangle.toArray()[0].x) > (sizeX - maxRectangle.toArray()[2].x)) {
                strPosition = "in right";
            } else {
                strPosition = "in middle";
            }
            Log.d("NumU", Double.toString(maxRectangle.toArray()[0].y)
                    + " : " + Double.toString(sizeY - maxRectangle.toArray()[1].y));
            if ((maxRectangle.toArray()[0].y) < (sizeY - maxRectangle.toArray()[1].y)) {
                strPosition += " , " + "upper";
            } else if ((maxRectangle.toArray()[0].y) > (sizeY - maxRectangle.toArray()[1].y)) {
                strPosition += " , " + "lower";
            } else {
                strPosition += " , " + "middle";
            }
            Log.d("Position",strPosition);
        }
    }
    public static void rotate(Mat image, double angle) {
        //Calculate size of new matrix
        double radians = Math.toRadians(angle);
        double sin = Math.abs(Math.sin(radians));
        double cos = Math.abs(Math.cos(radians));
        int newWidth = (int) (image.width() * cos + image.height() * sin);
        int newHeight = (int) (image.width() * sin + image.height() * cos);
        // rotating image
        Point center = new Point(newWidth / 2, newHeight / 2);
        Mat rotMatrix = Imgproc.getRotationMatrix2D(center, angle, 1.0); //1.0 means 100 % scale
        Size size = new Size(newWidth, newHeight);
        Imgproc.warpAffine(image, image, rotMatrix, image.size());
    }
    private Mat rotationAffineTutorial(Mat mRgba){
        // assuming source image's with and height are a pair value:
        double centerX = Math.round(mRgba.width()/2);
        double centerY = Math.round(mRgba.height()/2);
        Point center = new Point(centerY,centerX);
        double angle = 90;
        double scale = 1.0;
        double ratio =  mRgba.height() / (double) mRgba.width();
        int rotatedHeight = (int) Math.round(mRgba.height());
        int rotatedWidth  = (int) Math.round(mRgba.height() * ratio);
        Mat mapMatrix = Imgproc.getRotationMatrix2D(center, angle, scale);
        Size rotatedSize = new Size(rotatedWidth, rotatedHeight);
        Mat mIntermediateMat = new Mat(rotatedSize, mRgba.type());
        Imgproc.warpAffine(mRgba, mIntermediateMat, mapMatrix, mIntermediateMat.size(), Imgproc.INTER_LINEAR);
        Mat ROI = mRgba.submat(0, mIntermediateMat.rows(), 0, mIntermediateMat.cols());
        mIntermediateMat.copyTo(ROI);
        return ROI;
    }
    int thresh = 50, N = 11;
    void findSquares( Mat image, List<MatOfPoint> squares )
    {
        squares.clear();
        Mat smallerImg=new Mat(new Size(image.width()/2, image.height()/2),image.type());
        Mat gray = new Mat(image.size(),image.type());
        Mat gray0 = new Mat(image.size(),CvType.CV_8U);
        // down-scale and upscale the image to filter out the noise
        Imgproc.pyrDown(image, smallerImg, smallerImg.size());
        Imgproc.pyrUp(smallerImg, image, image.size());
        // find squares in every color plane of the image
        for( int c = 0; c < 3; c++ )
        {
            extractChannel(image, gray, c);
            // try several threshold levels
            for( int l = 1; l < N; l++ )
            {
                //Cany removed... Didn't work so well
                //Imgproc.threshold(gray, gray0, (l+1)*255/N, 255, Imgproc.THRESH_BINARY);
                List<MatOfPoint> contours=new ArrayList<MatOfPoint>();
                // find contours and store them all as a list
                Imgproc.findContours(gray0, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
                MatOfPoint approx = new MatOfPoint();
                // test each contour
                for( int i = 0; i < contours.size(); i++ )
                {
                    // approximate contour with accuracy proportional
                    // to the contour perimeter
                    approx = approxPolyDP(contours.get(i),  Imgproc.arcLength(new MatOfPoint2f(contours.get(i).toArray()), true)*0.02, true);
                    // square contours should have 4 vertices after approximation
                    // relatively large area (to filter out noisy contours)
                    // and be convex.
                    // Note: absolute value of an area is used because
                    // area may be positive or negative - in accordance with the
                    // contour orientation
                    if( approx.toArray().length == 4 &&
                            Math.abs(Imgproc.contourArea(approx)) > 1000 &&
                            Imgproc.isContourConvex(approx) )
                    {
                        double maxCosine = 0;
                        for( int j = 2; j < 5; j++ )
                        {
                            // find the maximum cosine of the angle between joint edges
                            double cosine = Math.abs(angle(approx.toArray()[j%4], approx.toArray()[j-2], approx.toArray()[j-1]));
                            maxCosine = Math.max(maxCosine, cosine);
                        }
                        // if cosines of all angles are small
                        // (all angles are ~90 degree) then write quandrange
                        // vertices to resultant sequence
                        if( maxCosine < 0.3 ) {
                            squares.add(approx);
                        }
                    }
                }
            }
        }
    }
    void extractChannel(Mat source, Mat out, int channelNum) {
        List<Mat> sourceChannels=new ArrayList<Mat>();
        List<Mat> outChannel=new ArrayList<Mat>();
        Core.split(source, sourceChannels);
        outChannel.add(new Mat(sourceChannels.get(0).size(),sourceChannels.get(0).type()));
        Core.mixChannels(sourceChannels, outChannel, new MatOfInt(channelNum,0));
        Core.merge(outChannel, out);
    }
    MatOfPoint approxPolyDP(MatOfPoint curve, double epsilon, boolean closed) {
        MatOfPoint2f tempMat=new MatOfPoint2f();
        Imgproc.approxPolyDP(new MatOfPoint2f(curve.toArray()), tempMat, epsilon, closed);
        return new MatOfPoint(tempMat.toArray());
    }
    @Override
    protected void onPause() {
        super.onPause();
        if (cameraBridgeViewBase != null) {
            cameraBridgeViewBase.disableView();
        }
    }
    @Override
    protected void onResume() {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Toast.makeText(getApplicationContext(), "There is a problem", Toast.LENGTH_SHORT).show();
        } else {
            baseLoaderCallback.onManagerConnected(BaseLoaderCallback.SUCCESS);
        }
    }
    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (cameraBridgeViewBase != null) {
            cameraBridgeViewBase.disableView();
        }
    }
    private static double angle(Point pt1, Point pt2, Point pt0) {
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt2.x - pt0.x;
        double dy2 = pt2.y - pt0.y;
        return (dx1 * dx2 + dy1 * dy2) / Math.sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
    }
    private List<MatOfPoint> findBiggestRactangle(List<MatOfPoint> squares){
        List<MatOfPoint> maxList = new ArrayList<>();
        double maxDimen = 0;
        double result = 0;
        int positionMax = -1;
        for(int i = 0 ; i < squares.size(); i++ ){
            result = (squares.get(i).toArray()[1].y - squares.get(i).toArray()[0].y)
                    * (squares.get(i).toArray()[2].x - squares.get(i).toArray()[0].x);
            if (result > maxDimen){
                maxDimen = result;
                positionMax = i;
            }
        }
        if (positionMax != -1){
            maxList.add(squares.get(positionMax));
            Log.d("MaxPo","" +
                    squares.get(positionMax).toArray()[0].x + " : " +
                    squares.get(positionMax).toArray()[0].y
            );
            return maxList;
        }
        return null;
    }
    public Mat drawDetectedMarkers(Mat mRgba) {
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat arucoIDs = new Mat();
        Aruco.drawDetectedMarkers(mRgba, corners, arucoIDs, new Scalar(0, 255, 0));
        //Imgproc.putText(mRgba, "Aruco:"+ **arucoIDs.get(1,1)**, new Point(30, 30), 3, 1, new Scalar(255, 0, 0, 255), 1);
        Imgproc.putText(mRgba, "Aruco:", new Point(30, 80), 3, 0.5, new Scalar(255, 0, 0, 255), 1);
        return arucoIDs;

    }
    private Mat detectARMarker(Mat inputImage){
        Log.d(TAG,"Detect AR");
        Boolean status = false;
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        String value = null;
        String value2 = null;
        DetectorParameters parameters = null;
        Mat newImg = null;
        int maxLoop = 5;
        int loop = 0;
        while (value == null && loop < maxLoop) {
            try {
                parameters = DetectorParameters.create();
                Aruco.detectMarkers(inputImage, dictionary, corners, markerIds, parameters);
                Aruco.drawDetectedMarkers(inputImage, corners,newImg , new Scalar(0, 255, 0));
                value = Arrays.toString(markerIds.get(0,0));
                value2 = String.valueOf(markerIds.nativeObj);
                Log.d(TAG,"value= " + value);
                Log.d(TAG,"value=2= " + value2);

            }
            catch (Exception e){
                Log.d(TAG,"AR Error");
                Log.d(TAG,e.getMessage());
            }
            loop++;
        }
        if (value != null) {

            status = true;
        }
        else {
            status = false;
        }

        return newImg;
    }
   // private void rotateImage(srcMat){
   //     Point center = new Point(x,y);
   //     double angle = 90;
   //     double scale = 1.0;
   //     Mat mapMatrix = Imgproc.getRotationMatrix2D(center, angle, scale);
   //     Imgproc.warpAffine(srcMat, srcMat, mapMatrix, Imgproc.INTER_LINEAR);
   // }
   // private void setLabel(Mat im, String label, MatOfPoint contour) {
   //     int fontface = Core.FONT_HERSHEY_SIMPLEX;
   //     double scale = 3;//0.4;
   //     int thickness = 3;//1;
   //     int[] baseline = new int[1];
   //     Size text = Imgproc.getTextSize(label, fontface, scale, thickness, baseline);
   //     Rect r = Imgproc.boundingRect(contour);
   //     Point pt = new Point(r.x + ((r.width - text.width) / 2),r.y + ((r.height + text.height) / 2));
   //     Imgproc.putText(im, label, pt, fontface, scale, new Scalar(255, 0, 0), thickness);
   // }
}

*/

public class MainActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        YourService ys = new YourService();
        ys.runPlan1();
    }

    public static void main(String[] args) {
        String tx = "qua_z, -0.4750849972774904";
        double x =  0.37473758240533916;
        double y =  -0.5101320626024768;
        double z = -0.2914347506841285 ;

        double mag = Math.sqrt(Math.pow(x,2) + Math.pow(y,2) + Math.pow(z,2));
        double w =Math.asin(mag);
        long a = 10;
        System.out.println(Long.toString(a));
        System.out.println(w);
    }
}
