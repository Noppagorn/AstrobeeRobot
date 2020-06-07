package jp.jaxa.iss.kibo.rpc.defaultapk;


import android.graphics.Bitmap;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.FormatException;
import com.google.zxing.LuminanceSource;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.datamatrix.DataMatrixReader;
import com.google.zxing.qrcode.QRCodeReader;


import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;

import java.io.ByteArrayOutputStream;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import gov.nasa.arc.astrobee.android.gs.MessageType;
import gov.nasa.arc.astrobee.types.Vec3d;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import jp.jaxa.iss.kibo.rpc.api.types.PointCloud;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import sensor_msgs.PointCloud2;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Aruco;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import static android.content.ContentValues.TAG;
public class YourService extends KiboRpcService {
    double navPox = 0;
    double navPoy = 0;
    double navPoz = 0;
    String flow = "flow";
    String ground = "ground";
    String side = "side";
    int[] intArray = new int[1280 * 960];
    Reader reader = new QRCodeReader();

    public double norm(double[] V) {
        double squareX = Math.pow(V[0], 2);
        double squareY = Math.pow(V[1], 2);
        double squareZ = Math.pow(V[2], 2);
        return Math.sqrt(squareX + squareY + squareZ);
    }

    // Note: Turn vector into unit vector
    public double[] computeUnitVector(double[] V) {
        double[] U = new double[3];
        double size = norm(V);

        U[0] = V[0] / size;
        U[1] = V[1] / size;
        U[2] = V[2] / size;

        return U;
    }

    // Note: Turn quaternion into unit vector
    public double[] quaternionToVector(double x, double y, double z, double w) {
        double[] V = new double[3];

        V[0] = 2 * (x * z - w * y);
        V[1] = 2 * (y * z + w * x);
        V[2] = 1 - 2 * (x * x + y * y);

        return computeUnitVector(V);
    }

    public double[] findTargetPoint(double x, double y, double z,
                                    double qx, double qy, double qz, double qw) {
        double[] direction = quaternionToVector(qx, qy, qz, qw);
        double[] target = new double[3];
        // Ty = -10.2
        target[1] = -10.2;
        // calculate vector size
        double size = (target[1] - y) / direction[1];
        // calulate Tx, Tz
        target[0] = x + size * direction[0];
        target[2] = z + size * direction[2];
        // remove offset
        target[1] += 0.6;

        return target;
    }




    @Override
    protected void runPlan1(){

        String p1_1 = "";
        Log.d(TAG,"Start Simulator");
        int w = 1280, h = 960;
        Bitmap.Config conf = Bitmap.Config.ARGB_8888; // see other conf types
        Bitmap bmp = Bitmap.createBitmap(w, h, conf); // this creates a MUTABLE bitmap
        //mulScanPrepare(bmp);
        detectBarCode(bmp);
        intArray = new int[1280 * 960];
        Log.d(TAG,"Finish prepare");
        api.judgeSendStart();

        Log.d(TAG,"start scan 1");
        moveToWrapper2(11.52 + navPox, -5.69 + navPoy, 4.5 + navPoz, 0, 0, 0, 1);

        p1_1 = scanBarcode(11.52, -5.69 + navPoy, 4.5 + navPoz, 0, 0, 0, 1, 0, "value p1-1","side");
        Log.d(TAG,"p1_1 = " + p1_1);
        Log.d(TAG,"start scan 2");

        moveToWrapper2(11 + navPox, -6 + navPoy, 5.55 + navPoz, 0, -0.7071068, 0, 0.7071068);
        Log.d(TAG,"IMU linear" + api.getImu().getLinearAcceleration().toString());
        Log.d(TAG,"IMU Angular" + api.getImu().getAngularVelocity().toString());
        Log.d(TAG,"IMU Orientation" + api.getImu().getOrientation().toString());
        String p1_2 = scanBarcode(11 + navPox, -6 + navPoy, 5.55 + navPoz, 0, -0.7071068, 0, 0.7071068,1,"values p1-2","ground");
        Log.d(TAG,"p1_2 = " + p1_2);
        Log.d(TAG,"finish scan 2");
        Log.d(TAG,"start scan 3");

        Log.d(TAG,"start detect 3");
        moveToWrapper2(11, -5.5, 4.33, 0, 0.7071068, 0, 0.7071068);
        String p1_3 = scanBarcode(11, -5.5, 4.33, 0, 0.7071068, 0, 0.7071068,2,"value p1-3","flow");
        Log.d(TAG,"p1_3 = " + p1_3);
        Log.d(TAG,"finish scan 3");

        Log.d(TAG,"movePass1");
        moveToWrapper2(10.27, -6.8, 5.0, 0, 0, 0, 1);
        Log.d(TAG,"movePass2");
        moveToWrapper2(11.2, -6.8, 5.0, 0, 0, -0.7071068, 0.7071068);
        Log.d(TAG,"finishMovePass");

        moveToWrapper2(11.5, -8, 5, 0, 0, 0, 1);
        String p2_2 = scanBarcode(11.5, -8, 5, 0, 0, 0, 1,4,"value p2-2","");
        Log.d(TAG,"finnish scan 2_2") ;
        Log.d(TAG,"p2_2 = " + p2_2);

        moveToWrapper2(10.30, -7.5, 4.7, 0, 0, 1, 0);
        String p2_1 = scanBarcode(10.30, -7.5, 4.7, 0, 0, 1, 0,3,"value p2-1","");
        Log.d(TAG,"finnish scan 2_1") ;
        Log.d(TAG,"p2_1 = " + p2_1);

        moveToWrapper2(11, -7.7, 5.55, 0, -0.7071068, 0, 0.7071068);
        String p2_3 = scanBarcode(11, -7.7, 5.55, 0, -0.7071068, 0, 0.7071068,5,"value p2-3","ground");
        Log.d(TAG,"finnish scan 2_3") ;
        Log.d(TAG,"p2_3 = " + p2_3);
        Log.d(TAG,"finnish phase 2") ;

        moveToWrapper2(11, -7.7, 4.5, 0, 0, -0.7071068,  0.7071068);
        Log.d(TAG,"b1");
        moveToWrapper2(11, -9.2, 4.5, 0, 0, -0.7071068,  0.7071068);
        Log.d(TAG,"b2");
        //Double.p
        double doubleP1_1 = Double.parseDouble(p1_1.split(" ")[1]);
        double doubleP1_2 = Double.parseDouble(p1_2.split(" ")[1]);
        double doubleP1_3 = Double.parseDouble(p1_3.split(" ")[1]);
        double doubleP2_1 = Double.parseDouble(p2_1.split(" ")[1]);
        double doubleP2_2 = Double.parseDouble(p2_2.split(" ")[1]);
        double doubleP2_3 = Double.parseDouble(p2_3.split(" ")[1]);
        try {

            Log.d(TAG,"" + findW2(doubleP2_1,doubleP2_2,doubleP2_3));
            moveToWrapper2(doubleP1_1,doubleP1_2,doubleP1_3,doubleP2_1,
                    doubleP2_2,doubleP2_3,findW2(doubleP2_1,
                            doubleP2_2,doubleP2_3));
            Log.d(TAG,"DetectAR");
            double status = detectARMarker2();
            if (status == -1.0){
                Log.d(TAG,"Not correct W");
                moveToWrapper2(doubleP1_1,doubleP1_2,doubleP1_3,doubleP2_1,
                        doubleP2_2,doubleP2_3,-1.0 * findW2(doubleP2_1,
                                doubleP2_2,doubleP2_3));
                status = detectARMarker2();
            }
            Log.d(TAG,"DetectAR2");

            //String p3 = scanBarcode(doubleP1_1,doubleP1_2,doubleP1_3,doubleP2_1,
             //       doubleP2_2,doubleP2_3,0,5,"P3","");
            //Log.d(TAG,"P3 = " + p3.toString());
        }
        catch (Exception e){
            Log.d(TAG,"ERROR!!" + e.getMessage());
        }
        Log.d(TAG,"Move Target");
        double[] target = findTargetPoint(doubleP1_1,doubleP1_2,doubleP1_3,doubleP2_1,
                doubleP2_2,doubleP2_3,findW2(doubleP2_1,
                        doubleP2_2,doubleP2_3));
        Log.d(TAG,"Target values = " + Double.toString(target[0])  + " "+ Double.toString(target[1])
                + " " +  Double.toString(target[2]));
        moveToWrapper2(target[0],target[1],target[2],0, 0, -0.7071068, 0.7071068);

        api.laserControl(true);
        api.judgeSendFinishISS();
        api.judgeSendFinishSimulation();
    }
    @Override
    protected void runPlan2(){

    }
    @Override
    protected void runPlan3(){
    }
    private double findmagnitude(double qx,double qy,double qz){
        return Math.sqrt(Math.pow(qx,2) + Math.pow(qy,2) + Math.pow(qz,2));
    }
    private double findW(double qx,double qy,double qz){
        return Math.asin(findmagnitude(qx,qy,qz));
    }
    private double findW2(double qx,double qy,double qz){
        return Math.sqrt(1 - Math.pow(qx,2) + Math.pow(qy,2) + Math.pow(qz,2));
    }
    private void moveLittle(double pos_x, double pos_y, double pos_z,
                            double qua_x, double qua_y, double qua_z,
                            double qua_w){
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);
        Result result = api.moveTo(point, quaternion, true);
        Log.d(TAG,"moveLittle");
        Log.d(TAG,"move Message" + result.getMessage());
        Log.d(TAG,"move Status" + result.getStatus());
    }
    private Result moveToWrapper2(double pos_x, double pos_y, double pos_z,
                                  double qua_x, double qua_y, double qua_z,
                                  double qua_w){
        final int LOOP_MAX = 5;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);
        Log.d(TAG,"movetoWrapper2");
        printPosition("mtw2",pos_x,pos_y,pos_z,qua_x,qua_y,qua_z,
                qua_w);
        Result result = api.moveTo(point, quaternion, true);
        Log.d(TAG,"mt Finish");

        int loopCounter = 0;
        while(!result.hasSucceeded() && loopCounter < LOOP_MAX){
            result = api.moveTo(point, quaternion, true);
            if (result.getStatus().toString().equals("EXEC_FAILED(3)")){
                printPosition("Move Fail",pos_x,pos_y,pos_z,qua_x,qua_y,qua_z,qua_w);
                printAllPosition("----InMove----");
            }
            Log.d(TAG,"loopCounter : " + loopCounter);
            Log.d(TAG,"move Message" + result.getMessage());
            Log.d(TAG,"move Status" + result.getStatus());
            ++loopCounter;
        }
        return  result;
    }
    private void scanWhenNearQr(double pos_x, double pos_y, double pos_z,
                                double qua_x, double qua_y, double qua_z,
                                double qua_w){

    }
    private boolean scanWhenNear(double pos_x, double pos_y, double pos_z,
                                 double qua_x, double qua_y, double qua_z,
                                 double qua_w){
        boolean isNear = false;
        double x =  api.getTrustedRobotKinematics().getPosition().getX();
        double y =  api.getTrustedRobotKinematics().getPosition().getY();
        double z =  api.getTrustedRobotKinematics().getPosition().getZ();
        double qx = api.getTrustedRobotKinematics().getOrientation().getX();
        double qy = api.getTrustedRobotKinematics().getOrientation().getY();
        double qz = api.getTrustedRobotKinematics().getOrientation().getZ();
        double qw = api.getTrustedRobotKinematics().getOrientation().getW();

        double mean = (area(pos_x,x) + area(pos_y,y) + area(pos_z,z))/3;
        if (mean < 0.1){
            isNear = true;
        }
        return isNear;

    }
    private boolean onlyScan(int no){
        Bitmap snapshot = api.getBitmapNavCam();
        String value = detectBarCode(snapshot);
        Log.d(TAG,"onlyScan");
        if (value != null){
            api.judgeSendDiscoveredQR(no,value);
            Log.d(TAG,"true");
            return true;
        }
        else {
            Log.d(TAG,"false");
            return false;
        }
    }
    private String mulScanPrepare(Bitmap snapshot){
        int maxLoop = 2;
        int loop = 0;
        String value = null;
        while (loop < maxLoop){
            detectBarCode(snapshot);
            loop++;
        }
        return value;
    }
    private String mulScan(){
        int maxLoop = 3;
        int loop = 0;
        String value = null;
        Bitmap snapshot = null;
        while (value == null && loop < maxLoop){
            snapshot = api.getBitmapNavCam();
            value = detectBarCode(snapshot);
            loop++;
        }
        return value;
    }
    private void moveToRelativeWrapper2(double pos_x, double pos_y, double pos_z,
                                        double qua_x, double qua_y, double qua_z,
                                        double qua_w){
        Log.d(TAG,"Relative Position 1 :" + pos_x + " " + pos_y + " " + pos_z
                + " Qua " + qua_x + " " + qua_y + " " + qua_z + " " + qua_w);
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);
        Result result = api.relativeMoveTo(point, quaternion, true);
        Log.d(TAG,result.getStatus().toString()) ;
        Log.d(TAG,result.getMessage()) ;
        Log.d(TAG, "moveRel 1 Finnish");
    }
    private void moveToRelativeWrapper3(double pos_x, double pos_y, double pos_z,
                                        double qua_x, double qua_y, double qua_z,
                                        double qua_w){
        Log.d(TAG,"Relative Position :" + pos_x + " " + pos_y + " " + pos_z
                + " Qua " + qua_x + " " + qua_y + " " + qua_z + " " + qua_w);
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);
        int loop = 0;
        int loopMax = 3;
        Result result = api.relativeMoveTo(point, quaternion, true);
        while (!result.hasSucceeded() && loop < loopMax) {
            result = api.relativeMoveTo(point, quaternion, true);
            if (result.getStatus().toString().equals("EXEC_FAILED(3)")){
                printPosition("MoveRel Fail",pos_x,pos_y,pos_z,qua_x,qua_y,qua_z,qua_w);
                printAllPosition("----InRelMove----");
            }
            Log.d(TAG,"loopCounter : " + loop);
            Log.d(TAG,"move Message" + result.getMessage());
            Log.d(TAG,"move Status" + result.getStatus());
            ++loop;
        }
        Log.d(TAG, "moveRel Finnish");
    }
    private double converTwodicimal(double x){
        DecimalFormat df = new DecimalFormat("#.##");
        x = Double.valueOf(df.format(x));
        return x;
    }
    private Point relativeStablePoint(Point targetPoint, Point curentPoint ){
        double X_area = area(converTwodicimal(targetPoint.getX()),converTwodicimal(curentPoint.getX()));
        double Y_area = area(converTwodicimal(targetPoint.getY()),converTwodicimal(curentPoint.getY()));
        double Z_area = area(converTwodicimal(targetPoint.getZ()),converTwodicimal(curentPoint.getZ()));
        printPosition("Point relative stable target",
                targetPoint.getX(),targetPoint.getY(),targetPoint.getZ(),0,0,0,0);
        printPosition("Point relative stable current",
                curentPoint.getX(),curentPoint.getY(),curentPoint.getZ(),0,0,0,0);
        if (targetPoint.getX() < curentPoint.getX()){
            X_area = -X_area;
        }
        if (targetPoint.getY() < curentPoint.getY()){
            Y_area = -Y_area;
        }
        if (targetPoint.getZ() < curentPoint.getZ()){
            Z_area = -Z_area;
        }
        printPosition("Point relative stable area",
                X_area,Y_area,Z_area,0,0,0,0);
        return new Point(X_area,Y_area,Z_area);
    }
    private Quaternion relativeStableQuaternion(Quaternion targetQua, Quaternion currentQua){
        printPosition("relative stable target",
                0,0,0,(double) targetQua.getX()
                ,(double) targetQua.getY(),(double) targetQua.getZ(),(double) targetQua.getW());
        printPosition("relative stable current",
                0,0,0,(double) currentQua.getX()
                ,(double) currentQua.getY(),(double) currentQua.getZ(),(double) currentQua.getW());

        Quaternion inverse = QuaternionUtill.Inverse(currentQua);
        Quaternion mul = QuaternionUtill.Multiply(inverse,targetQua);
        printPosition("relative stable result",
                0,0,0,(double) mul.getX()
                ,(double) mul.getY(),(double) mul.getZ(),(double) mul.getW());
        return mul;
    }

    private double area(double x , double y){
        return Math.abs(x - y);
    }

    private String scanBarcode(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w,int no,String title,String dimen){
        String value = scanBarcode2(pos_x,pos_y,pos_z,qua_x,qua_y,qua_z,qua_w,no,dimen);
        Log.d(TAG,title);
        return value;
    }

    private String scanBarcode2(double pos_x, double pos_y, double pos_z,
                                double qua_x, double qua_y, double qua_z,
                                double qua_w,int no,String dimen){
        Result result = null;
        Point updatePoint = new Point();
        Quaternion updateQuaternion = new Quaternion();
        int loopMax = 3;
        int loop = 0;
        Bitmap snapshot = null;
        String value = null;
        printAllPosition("position Scan");
        //snapshot = api.getBitmapNavCam();
        //value = detectBarCode(snapshot);
        value = mulScan();
        while (value == null && loop < loopMax) {
            //moveToWrapper2(pos_x,pos_y,pos_z,qua_x,qua_y,qua_z,qua_w);
            Log.d(TAG,"Confidence : "+api.getTrustedRobotKinematics().getConfidence());
            double x = api.getTrustedRobotKinematics().getPosition().getX();
            double y = api.getTrustedRobotKinematics().getPosition().getY();
            double z = api.getTrustedRobotKinematics().getPosition().getZ();
            double qx = api.getTrustedRobotKinematics().getOrientation().getX();
            double qy = api.getTrustedRobotKinematics().getOrientation().getY();
            double qz = api.getTrustedRobotKinematics().getOrientation().getZ();
            double qw = api.getTrustedRobotKinematics().getOrientation().getW();
            printAllPosition("---before sep---");

            updateQuaternion = relativeStableQuaternion(new Quaternion((float) qua_x, (float) qua_y
                    , (float) qua_z, (float) qua_w), new Quaternion((float) qx, (float) qy
                    , (float) qz, (float) qw));
            moveToRelativeWrapper3(0,0,0,
                    updateQuaternion.getX(),updateQuaternion.getY(),
                    updateQuaternion.getZ(),updateQuaternion.getW());
            //snapshot = api.getBitmapNavCam();
            //value = detectBarCode(snapshot);
            value = mulScan();

            if (value == null)
            {
                if (dimen.equals("side")) {
                    pos_x += 0.01;
                }
                else if (dimen.equals("right")) {
                    pos_x -= 0.002;
                }
                else if (dimen.equals("ground")) {
                    pos_z += 0.002;
                }
                else if (dimen.equals("flow")) {
                    pos_z -= 0.002;
                }

            }
            moveLittle(pos_x, pos_y, pos_z, qua_x, qua_y, qua_z, qua_w);
            loop++;
        }
        if (value != null) {
            api.judgeSendDiscoveredQR(no , value);
            System.out.println("valuesQR" + value);
        }
        else{
            System.out.println("valuesQR = null");
        }
        return value;
    }

    private void printPosition(String label,double x,double y,double z
            , double qx,double qy, double qz, double qw){
        try {

            Log.d(TAG, "all Pos");
            Log.d(TAG, label + " ; Position = " + x + " "
                    + y + " " + z + " " + "; Quan " + qx + " " + qy + " " + qz + " " + qw);
        }
        catch (Exception e){
            Log.d(TAG,"Exception printPosition");
        }
    }
    private void printAllPosition(String lable){
        Log.d(TAG,lable);
        double x =  api.getTrustedRobotKinematics().getPosition().getX();
        double y =  api.getTrustedRobotKinematics().getPosition().getY();
        double z =  api.getTrustedRobotKinematics().getPosition().getZ();
        double qx = api.getTrustedRobotKinematics().getOrientation().getX();
        double qy = api.getTrustedRobotKinematics().getOrientation().getY();
        double qz = api.getTrustedRobotKinematics().getOrientation().getZ();
        double qw = api.getTrustedRobotKinematics().getOrientation().getW();
        Log.d(TAG,"IMU linear" + api.getImu().getLinearAcceleration().toString());
        Log.d(TAG,"IMU Angular" + api.getImu().getAngularVelocity().toString());
        Log.d(TAG,"IMU Orientation" + api.getImu().getOrientation().toString());
        printPosition("moving",x,y, z,qx
                ,qy,qz,qw);
        Log.d(TAG,"<" + lable + ">");
    }

    private void getImuValue(){
        Log.d(TAG,"getImuValue");
        Log.d(TAG,"IMU linear" + api.getImu().getLinearAcceleration().toString());
        Log.d(TAG,"IMU Angular" + api.getImu().getAngularVelocity().toString());
        Log.d(TAG,"IMU Orientation" + api.getImu().getOrientation().toString());
    }
    private String Convert(Mat imgs){
        //QRCodeDetector detectAndDecode = new QRCodeDetector();
        //String value = detectAndDecode.detectAndDecode(imgs);
        //return value;
        return "";
    }

    String detectBarCode(Bitmap bitmap) {
        Log.d(TAG,"Sc1");

        Log.d(TAG,"sc2");
        bitmap.getPixels(intArray, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());
        Log.d(TAG,"sc2.5");
        LuminanceSource source = new RGBLuminanceSource(bitmap.getWidth(), bitmap.getHeight(), intArray);
        Log.d(TAG,"Sc3");

        try {
            com.google.zxing.Result result = reader.decode(new BinaryBitmap(new HybridBinarizer(source)));
            Log.d(TAG,"Sc4");
            return result.getText();
        } catch (NotFoundException e) {
            Log.d(TAG,"QRnotfound");
            return null;
        } catch (ChecksumException e) {
            Log.d(TAG,"QRCheckSum");
            return null;
        } catch (FormatException e) {
            Log.d(TAG,"QRFormat");
            return null;
        }catch (Exception e){
            Log.d(TAG,"QRException");
            return null;
        }
    }
    private double detectARMarker2(){
        Log.d(TAG,"detecting AR TAG");
        Dictionary dictionary =  Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        Mat gimage = null;
        Mat ids = new Mat();
        List<Mat> corners = new ArrayList<>();
        int maxLoop = 5;
        int loop = 0;
        double makerID = -1.0;
        while (makerID == -1.0 && loop < maxLoop) {
            gimage = api.getMatNavCam();
            Aruco.detectMarkers(gimage, dictionary, corners, ids);
            Log.d(TAG,"AR Cornor Size " +  corners.size());
            if (corners.size() > 0) {
                Log.d(TAG,"AR = " +  Arrays.toString(ids.get(0, 0)));
                makerID = ids.get(0, 0)[0];

            }
            loop++;
        }
        if (makerID != -1.0){
            Log.d(TAG,"M1");
            //api.judgeSendDiscoveredAR(Double.toString(makerID));
            api.judgeSendDiscoveredAR(Long.toString(Math.round(makerID)));
            Log.d(TAG,"round "  + Long.toString(Math.round(makerID)));
        }
        return makerID;
    }
    private boolean detectARMarker(){
        Log.d(TAG,"Detect AR");
        Boolean status = false;
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        String value = null;
        String value2 = null;
        Mat inputImage = null;
        DetectorParameters parameters = null;
        int maxLoop = 5;
        int loop = 0;
        while (value == null && loop < maxLoop) {
            try {
                inputImage = api.getMatNavCam();
                parameters = DetectorParameters.create();
                Aruco.detectMarkers(inputImage, dictionary, corners, markerIds, parameters);
                value = String.valueOf(markerIds.get(0,0)[0]);
                value2 = String.valueOf(markerIds.nativeObj);
                Log.d(TAG,"value in loop " +  value);
                Log.d(TAG,"value2 in loop " +  value2);
            }
            catch (Exception e){
                Log.d(TAG,"AR Error");
                Log.d(TAG,e.getMessage());
            }
            loop++;
        }
        if (value != null) {
            api.judgeSendDiscoveredAR(value);
            api.judgeSendDiscoveredAR(value2);
            status = true;
        }
        else {
            status = false;
        }
        Log.d(TAG,"value = " + value);
        Log.d(TAG,"value2 = " + value2);
        return status;
    }
}