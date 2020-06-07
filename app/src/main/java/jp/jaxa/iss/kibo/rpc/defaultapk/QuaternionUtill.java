package jp.jaxa.iss.kibo.rpc.defaultapk;

import gov.nasa.arc.astrobee.types.Quaternion;
public class QuaternionUtill {
    public static Quaternion Inverse(Quaternion quaternion)
    {
        double num2 = (((quaternion.getX() * quaternion.getX()) + (quaternion.getY() * quaternion.getY())) + (quaternion.getZ() * quaternion.getZ())) + (quaternion.getW() * quaternion.getW());
        double num = 1.0 / num2;

        Quaternion quaternion2 = new Quaternion((float) (-1 * quaternion.getX() * num)
                , (float) (-1 * quaternion.getY() * num)
                ,  (float) (-1 * quaternion.getZ() * num)
                ,(float) ( -1 * quaternion.getW() * num)
        );
        return quaternion2;
    }
    public static Quaternion Multiply(Quaternion quaternion1, Quaternion quaternion2)
    {

        double x = quaternion1.getX();
        double y = quaternion1.getY();
        double z = quaternion1.getZ();
        double w = quaternion1.getW();
        double num4 = quaternion2.getX();
        double num3 = quaternion2.getY();
        double num2 = quaternion2.getZ();
        double num = quaternion2.getW();
        double num12 = (y * num2) - (z * num3);
        double num11 = (z * num4) - (x * num2);
        double num10 = (x * num3) - (y * num4);
        double num9 = ((x * num4) + (y * num3)) + (z * num2);
        Quaternion quaternion = new Quaternion(
                (float)(((x * num) + (num4 * w)) + num12),
                (float)(((y * num) + (num3 * w)) + num11),
                (float)(((z * num) + (num2 * w)) + num10),
                (float)((w * num) - num9));
        return quaternion;
    }
}
class Main {
    public static void main(String[] args) {
        Quaternion m = new Quaternion(1, 0, 0, 0);

        Quaternion m1 = new Quaternion(-1, 0, 0, 0);
        Quaternion inver = QuaternionUtill.Inverse(m);
        Quaternion mul = QuaternionUtill.Multiply(inver,m1);
        System.out.println(inver);
        System.out.println(mul);
    }
}
