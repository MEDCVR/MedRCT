using System;
using UnityEngine;
using Medcvr.DvrkPlanning;

namespace Medcvr.DvrkPlanning.Psm
{
public static class Kinematics
{
    public static class Data
    {
        public const int numLinks = 7;

        public const float lRcc = 0.4318f;  // From dVRK documentation
        public const float lTool = 0.4162f; // From dVRK documentation
        public const float lPitch2yaw = 0.0091f;  // Fixed length from the palm joint to the pinch joint
        public const float lYaw2ctrlpnt = 0.0102f;  // Fixed length from the pinch joint to the pinch tip
        // Delta between tool tip and the Remote Center of Motion
        public const float lTool2rcm_offset = lRcc - lTool;

        private static Dh[] kinematics = new Dh[] {
            new Dh( Global.PI_2,       0.0f,         0.0f,  Global.PI_2, Dh.JointType.REVOLUTE),
            new Dh(-Global.PI_2,       0.0f,         0.0f, -Global.PI_2, Dh.JointType.REVOLUTE),
            new Dh( Global.PI_2,       0.0f,         0.0f,       -lRcc, Dh.JointType.PRISMATIC),
            new Dh(        0.0f,       0.0f,        lTool,         0.0f, Dh.JointType.REVOLUTE),
            new Dh(-Global.PI_2,       0.0f,         0.0f, -Global.PI_2, Dh.JointType.REVOLUTE),
            new Dh(-Global.PI_2, lPitch2yaw,         0.0f, -Global.PI_2, Dh.JointType.REVOLUTE),
            new Dh(-Global.PI_2,       0.0f, lYaw2ctrlpnt,  Global.PI_2, Dh.JointType.REVOLUTE)};

        public static Dh GetDh(int linkNum)
        {
            if(linkNum > numLinks)
            {
                throw new InvalidOperationException("linkNum must be <= numLinks");
            }
            return kinematics[linkNum];
        }
    }

    public static Matrix4x4 ComputeFk(float[] activeJointPos, int linkNumOutput = 7)
    {
        Matrix4x4 returnTransform = Matrix4x4.identity;
        if(activeJointPos.Length > Data.numLinks)
        {
            throw new InvalidOperationException("activeJointPos.Length must be less than " + Data.numLinks.ToString());
            return returnTransform;
        }
        if(activeJointPos.Length > linkNumOutput)
        {
            throw new InvalidOperationException("activeJointPos.Length must be less than " + Data.numLinks.ToString());
            return returnTransform;
        }
        for(int i = 0; i < activeJointPos.Length; i++)
        {
            returnTransform =
                returnTransform *
                Data.GetDh(i).ToMat(activeJointPos[i]);
        }
        for(int i = activeJointPos.Length; i < linkNumOutput; i++)
        {
            returnTransform =
                returnTransform *
                Data.GetDh(i).ToMat(0.0f);
        }

        return returnTransform;
    }

    public static float GetAngle(Vector3 vecA, Vector3 vecB, bool useUpVec = false, Vector3 upVec = new Vector3())
    {
        vecA = vecA.normalized;
        vecB = vecB.normalized;

        Vector3 crossAB =  Vector3.Cross(vecA, vecB);
        float vDot = Vector3.Dot(vecA, vecB);

        float angle = 0.0f;
        if (1.0f - vDot < 0.000001f)
        {
            angle = 0.0f;
        }
        else if (1.0f + vDot < 0.000001)
        {
            angle = MathF.PI;
        }
        else
        {
            angle = MathF.Acos(vDot);
        }

        if (useUpVec && MathF.Sign(Vector3.Dot(crossAB, upVec)) < 0.0f) // Check same direction
        {
            angle = -angle;
        }

        return angle;
    }

    public static Vector3 GetUpper3OfColumn(Matrix4x4 T, int i)
    {
        return new Vector3(
            T[0, i],
            T[1, i],
            T[2, i]);
    }

    // When all joint angles are zero, except insertion joint: {0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f}
    // r.Set(0.7071068f, 0.7071068f, 0.0f, 0.0f);
    // position = new Vector3(0.0f, 0.0f, -0.09670);
    // z-axis is facing down;
    // You should do ik below the RCM point!, Hence why insertion joint is 0.1f;
    public static float[] ComputeIK(Vector3 position, Quaternion r)
    {
        Matrix4x4 newMat = Matrix4x4.Rotate(r);
        newMat.SetColumn(3, new Vector4(position.x, position.y, position.z, 1.0f));
        return ComputeIK(newMat);
    }

    // Obtained from dvrk_planning psm.py which was obtained from:
    // https://github.com/collaborative-robotics/surgical_robotics_challenge/tree/master/scripts/surgical_robotics_challenge/kinematics
    public static float[] ComputeIK(Matrix4x4 T70)
    {
        Matrix4x4 T_PinchJoint_7 = Matrix4x4.identity;
        T_PinchJoint_7[2, 3] = -Data.lYaw2ctrlpnt;
        Matrix4x4 T_PinchJoint_0 = T70 * T_PinchJoint_7;

        Quaternion R_0_PinchJoint = T_PinchJoint_0.rotation;
        R_0_PinchJoint = Quaternion.Inverse(R_0_PinchJoint);

        Vector3 T_PinchJoint_0_p = GetUpper3OfColumn(T_PinchJoint_0, 3);

        Vector3 N_PalmJoint_PinchJoint = -1 * (R_0_PinchJoint * T_PinchJoint_0_p);
        N_PalmJoint_PinchJoint.x = 0.0f;
        N_PalmJoint_PinchJoint = N_PalmJoint_PinchJoint.normalized;

        Matrix4x4 T_PalmJoint_PinchJoint = Matrix4x4.identity;
        Vector3 T_PalmJoint_PinchJoint_p = N_PalmJoint_PinchJoint * Data.lPitch2yaw;
        T_PalmJoint_PinchJoint[0, 3] = T_PalmJoint_PinchJoint_p.x;
        T_PalmJoint_PinchJoint[1, 3] = T_PalmJoint_PinchJoint_p.y;
        T_PalmJoint_PinchJoint[2, 3] = T_PalmJoint_PinchJoint_p.z;

        Matrix4x4 T_PalmJoint_0 = T70 * T_PinchJoint_7 * T_PalmJoint_PinchJoint;

        Vector3 T_PalmJoint_0_p = GetUpper3OfColumn(T_PalmJoint_0, 3);
        float insertionDepth = T_PalmJoint_0_p.magnitude;

        // Angle calculations
        float xz_diag = Mathf.Pow(T_PalmJoint_0_p.x, 2.0f) + Mathf.Pow(T_PalmJoint_0_p.z, 2.0f);

        float j1 = Mathf.Atan2(T_PalmJoint_0_p.x, (-1.0f * T_PalmJoint_0_p.z));
        float j2 = -1 * Mathf.Atan2(T_PalmJoint_0_p.y, xz_diag);
        float j3 = insertionDepth + Data.lTool2rcm_offset;

        Vector3 T_7_0_R_UnitX = GetUpper3OfColumn(T70, 0);

        // Calculate j4
        Vector3 cross_palmlink_x7_0 = Vector3.Cross(T_7_0_R_UnitX,(T_PinchJoint_0_p - T_PalmJoint_0_p));
        Matrix4x4 T_3_0 = ComputeFk(new float[] {j1, j2, j3}, 3);
        Vector3 T_3_0_R_UnitY = GetUpper3OfColumn(T_3_0, 1);
        Vector3 T_3_0_R_UnitZ = GetUpper3OfColumn(T_3_0, 2);

        float j4 = GetAngle(cross_palmlink_x7_0, T_3_0_R_UnitY, true, -1.0f * T_3_0_R_UnitZ);

        Matrix4x4 T_4_3 = Data.GetDh(3).ToMat(j4);
        Matrix4x4 T_4_0 = T_3_0 * T_4_3;
        Vector3 T_4_0_R_UnitY = GetUpper3OfColumn(T_4_0, 1);
        Vector3 T_4_0_R_UnitZ = GetUpper3OfColumn(T_4_0, 2);
        float j5 = GetAngle(T_PinchJoint_0_p - T_PalmJoint_0_p, T_4_0_R_UnitZ, true, -1.0f * T_4_0_R_UnitY);

        Matrix4x4 T_5_4 = Data.GetDh(4).ToMat(j5);
        Matrix4x4 T_5_0 = T_4_0 * T_5_4;
        Vector3 T_5_0_R_UnitX = GetUpper3OfColumn(T_5_0, 0);
        Vector3 T_5_0_R_UnitY = GetUpper3OfColumn(T_5_0, 1);
        Vector3 T_7_0_R_UnitZ = GetUpper3OfColumn(T70, 2);

        float j6 = GetAngle(T_7_0_R_UnitZ, T_5_0_R_UnitX, true, -1.0f * T_5_0_R_UnitY);

        // return new float[]  {0.0f};
        return new float[] {j1, j2, j3, j4, j5, j6};
    }
}

}
