using System;
using UnityEngine;
using Medcvr.DvrkPlanning;

namespace Medcvr.DvrkPlanning.Psm
{
// For the psm kinematics explanation, follow dvrk_planning/src/dvrk_planning/kinematics/psm.py

public class LND400006 : SphericalWristToolParams
{
    public LND400006(float scale = 1.0f): base(
        0.4318f, // lRcc From dVRK documentation
        0.4162f, // lTool From dVRK documentation
        0.0091f, // lPitch2yaw From dVRK documentation
        0.0102f, // lYaw2Ctrlpnt From dVRK documentation
        scale){}
}

public class CF470049 : SphericalWristToolParams
{
    public CF470049(float scale = 1.0f): base(
        0.4318f, // lRcc From dVRK documentation
        0.4162f, // lTool From dVRK documentation
        0.0091f, // lPitch2yaw From dVRK documentation
        0.01977f, // lYaw2Ctrlpnt From CAD measurement
        scale){}
}

public class RTS470007 : SphericalWristToolParams
{
    public RTS470007(float scale = 1.0f): base(
        0.4318f, // lRcc From dVRK documentation
        0.4162f, // lTool From dVRK documentation
        0.0091f, // lPitch2yaw From dVRK documentation
        0.01041f, // lYaw2Ctrlpnt From CAD measurement
        scale){}
}

public class SphericalWristToolParams
{
    public readonly int numLinks = 7;
    private readonly Dh[] dhChain;

    public readonly float lRcc;
    public readonly float lTool;
    public readonly float lPitch2yaw;
    public readonly float lYaw2Ctrlpnt;
    public readonly float lTool2rcm_offset;

    public SphericalWristToolParams(
        float lRcc,
        float lTool,
        float lPitch2yaw,
        float lYaw2Ctrlpnt,
        float scale = 1.0f)
    {
        this.lRcc = lRcc * scale;
        this.lTool = lTool * scale;
        this.lPitch2yaw = lPitch2yaw * scale;  // Fixed length from the palm joint to the pinch joint
        this.lYaw2Ctrlpnt = lYaw2Ctrlpnt * scale;  // Fixed length from the pinch joint to the pinch tip
        // Delta between tool tip and the Remote Center of Motion
        lTool2rcm_offset = this.lRcc - this.lTool;

        dhChain = new Dh[] {
            new Dh( Global.PI_2,       0.0f,         0.0f,  Global.PI_2, Dh.JointType.REVOLUTE),
            new Dh(-Global.PI_2,       0.0f,         0.0f, -Global.PI_2, Dh.JointType.REVOLUTE),
            new Dh( Global.PI_2,       0.0f,         0.0f,       -this.lRcc, Dh.JointType.PRISMATIC),
            new Dh(        0.0f,       0.0f,        this.lTool,         0.0f, Dh.JointType.REVOLUTE),
            new Dh(-Global.PI_2,       0.0f,         0.0f, -Global.PI_2, Dh.JointType.REVOLUTE),
            new Dh(-Global.PI_2, this.lPitch2yaw,         0.0f, -Global.PI_2, Dh.JointType.REVOLUTE),
            new Dh(-Global.PI_2,       0.0f, this.lYaw2Ctrlpnt,  Global.PI_2, Dh.JointType.REVOLUTE)};
    }

    public Dh GetDh(int linkNum)
    {
        if(linkNum > numLinks)
        {
            throw new InvalidOperationException("linkNum must be <= numLinks");
        }
        return dhChain[linkNum];
    }
}

public class Kinematics
{
    private readonly SphericalWristToolParams kinematicsData;

    public Kinematics(SphericalWristToolParams parameters)
    {
        kinematicsData = parameters;
    }

    public Matrix4x4 ComputeFk(float[] activeJointPos, int linkNumOutput = 7)
    {
        Matrix4x4 returnTransform = Matrix4x4.identity;
        if(activeJointPos.Length > kinematicsData.numLinks)
        {
            throw new InvalidOperationException("activeJointPos.Length must be less than " + kinematicsData.numLinks.ToString());
        }
        if(activeJointPos.Length > linkNumOutput)
        {
            throw new InvalidOperationException("activeJointPos.Length must be less than " + kinematicsData.numLinks.ToString());
        }
        int activeJointPosLength = Math.Min(activeJointPos.Length, 6);
        for(int i = 0; i < activeJointPosLength; i++)
        {
            returnTransform =
                returnTransform *
                kinematicsData.GetDh(i).ToMat(activeJointPos[i]);
        }
        for(int i = activeJointPos.Length; i < linkNumOutput; i++)
        {
            returnTransform =
                returnTransform *
                kinematicsData.GetDh(i).ToMat(0.0f);
        }

        return returnTransform;
    }

    private static float GetAngle(Vector3 vecA, Vector3 vecB, bool useUpVec = false, Vector3 upVec = new Vector3())
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

    private static Vector3 GetUpper3OfColumn(Matrix4x4 T, int i)
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
    public float[] ComputeIK(Vector3 position, Quaternion rotation)
    {
        Matrix4x4 newMat = Matrix4x4.Rotate(rotation);
        newMat.SetColumn(3, new Vector4(position.x, position.y, position.z, 1.0f));
        return ComputeIK(newMat);
    }

    // Obtained from dvrk_planning psm.py which was obtained from:
    // https://github.com/collaborative-robotics/surgical_robotics_challenge/tree/master/scripts/surgical_robotics_challenge/kinematics
    public float[] ComputeIK(Matrix4x4 T70)
    {
        Matrix4x4 T_PinchJoint_7 = Matrix4x4.identity;
        T_PinchJoint_7[2, 3] = -kinematicsData.lYaw2Ctrlpnt;
        Matrix4x4 T_PinchJoint_0 = T70 * T_PinchJoint_7;

        Quaternion R_0_PinchJoint = T_PinchJoint_0.rotation;
        R_0_PinchJoint = Quaternion.Inverse(R_0_PinchJoint);

        Vector3 T_PinchJoint_0_p = GetUpper3OfColumn(T_PinchJoint_0, 3);

        Vector3 N_PalmJoint_PinchJoint = -1 * (R_0_PinchJoint * T_PinchJoint_0_p);
        N_PalmJoint_PinchJoint.x = 0.0f;
        N_PalmJoint_PinchJoint = N_PalmJoint_PinchJoint.normalized;

        Matrix4x4 T_PalmJoint_PinchJoint = Matrix4x4.identity;
        Vector3 T_PalmJoint_PinchJoint_p = N_PalmJoint_PinchJoint * kinematicsData.lPitch2yaw;
        T_PalmJoint_PinchJoint[0, 3] = T_PalmJoint_PinchJoint_p.x;
        T_PalmJoint_PinchJoint[1, 3] = T_PalmJoint_PinchJoint_p.y;
        T_PalmJoint_PinchJoint[2, 3] = T_PalmJoint_PinchJoint_p.z;

        Matrix4x4 T_PalmJoint_0 = T70 * T_PinchJoint_7 * T_PalmJoint_PinchJoint;

        Vector3 T_PalmJoint_0_p = GetUpper3OfColumn(T_PalmJoint_0, 3);
        float insertionDepth = T_PalmJoint_0_p.magnitude;

        // Angle calculations
        float xz_diag = Mathf.Sqrt(Mathf.Pow(T_PalmJoint_0_p.x, 2.0f) + Mathf.Pow(T_PalmJoint_0_p.z, 2.0f));

        float j1 = Mathf.Atan2(T_PalmJoint_0_p.x, (-1.0f * T_PalmJoint_0_p.z));
        float j2 = -1 * Mathf.Atan2(T_PalmJoint_0_p.y, xz_diag);
        float j3 = insertionDepth + kinematicsData.lTool2rcm_offset;

        Vector3 T_7_0_R_UnitX = GetUpper3OfColumn(T70, 0);

        // Calculate j4
        Vector3 cross_palmlink_x7_0 = Vector3.Cross(T_7_0_R_UnitX,(T_PinchJoint_0_p - T_PalmJoint_0_p));
        Matrix4x4 T_3_0 = ComputeFk(new float[] {j1, j2, j3}, 3);
        Vector3 T_3_0_R_UnitY = GetUpper3OfColumn(T_3_0, 1);
        Vector3 T_3_0_R_UnitZ = GetUpper3OfColumn(T_3_0, 2);

        float j4 = GetAngle(cross_palmlink_x7_0, T_3_0_R_UnitY, true, -1.0f * T_3_0_R_UnitZ);

        Matrix4x4 T_4_3 = kinematicsData.GetDh(3).ToMat(j4);
        Matrix4x4 T_4_0 = T_3_0 * T_4_3;
        Vector3 T_4_0_R_UnitY = GetUpper3OfColumn(T_4_0, 1);
        Vector3 T_4_0_R_UnitZ = GetUpper3OfColumn(T_4_0, 2);
        float j5 = GetAngle(T_PinchJoint_0_p - T_PalmJoint_0_p, T_4_0_R_UnitZ, true, -1.0f * T_4_0_R_UnitY);

        Matrix4x4 T_5_4 = kinematicsData.GetDh(4).ToMat(j5);
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
