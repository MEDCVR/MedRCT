using System;
using UnityEngine;

namespace Medcvr.DvrkPlanning
{
    public static class Global
    {
        public static float PI_2 = Mathf.PI/2;
    }


    public class Dh
    {
        public enum JointType
        {
            REVOLUTE = 0,
            PRISMATIC = 1
        }

        public enum Convention
        {
            STANDARD = 0,
            MODIFIED = 1
        }
        /*
        Returns a Matrix4x4 using Modified DH convention:
        alpha: Rad
        a: m
        d: m
        offset: m for PRISMATIC, Rad for revolute
        */
        private readonly float alpha;
        private readonly float a;
        private readonly float d;
        private readonly float offset;
        private readonly JointType joint_type;

        public Dh(float alpha,
            float a,
            float d,
            float offset,
            JointType joint_type)
        {
            this.alpha = alpha;
            this.a = a;
            this.d = d;
            this.offset = offset;
            this.joint_type = joint_type;
            if (joint_type != JointType.REVOLUTE &&
                joint_type != JointType.PRISMATIC)
            {
                throw new InvalidOperationException("joint_type must be PRISMATIC or REVOLUTE");
            }
        }

        // theta: m for PRISMATIC, Rad for revolute
        public Matrix4x4 ToMat(float theta = 0.0f)
        {
            float ca = MathF.Cos(alpha);
            float sa = MathF.Sin(alpha);
            float th = 0.0f;
            float local_d = d;
            if (joint_type == JointType.REVOLUTE){
               th = theta + offset;
            }
            else if (joint_type == JointType.PRISMATIC){
                local_d = local_d + offset + theta;
            }
            float ct = MathF.Cos(th);
            float st = MathF.Sin(th);

            Matrix4x4 newMat = new Matrix4x4();
            newMat.SetRow(0, new Vector4(     ct,     -st, 0.0f,       a));
            newMat.SetRow(1, new Vector4(st * ca, ct * ca,  -sa, -local_d * sa));
            newMat.SetRow(2, new Vector4(st * sa, ct * sa,   ca,  local_d * ca));
            newMat.SetRow(3, new Vector4(   0.0f,    0.0f, 0.0f,      1f));

            return newMat;
        }
    }
}
