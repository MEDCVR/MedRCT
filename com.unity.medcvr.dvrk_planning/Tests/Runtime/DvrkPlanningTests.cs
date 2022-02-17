using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using System;
using Medcvr.DvrkPlanning;

namespace Medcvr.DvrkPlanning.Tests
{
    public class DvrkPlanningTests
    {
        private float tol = 0.000001f;
        private Psm.Kinematics psmKinematics;

        [SetUp]
        public void SetUp()
        {
            psmKinematics = new Psm.Kinematics(new Psm.LND400006());
        }

        [Test]
        public void DebugComputeFkIk()
        {
            // float[] jps = new float[] {0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f};
            float[] jps = new float[] {0.2f, -0.2f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f};
            string debug_jps = "";
            foreach(float j in jps)
            {
                debug_jps = debug_jps + j.ToString() + " ";
            }
            Debug.Log("Input jps: " + debug_jps);

            Matrix4x4 output = psmKinematics.ComputeFk(jps, 7);
            Debug.Log("Fk: \n" + output);

            float[] outjps = psmKinematics.ComputeIK(output);
            Debug.Log("IK: ");
            string debug_ik = "";
            foreach(float j in outjps)
            {
                debug_ik = debug_ik + j.ToString() + " ";
            }
            Debug.Log(debug_ik);

            Matrix4x4 double_check_fk = psmKinematics.ComputeFk(outjps, 7);
            Debug.Log("Double check Fk: \n" + double_check_fk);
        }

        [Test]
        public void TestComputeFk()
        {
            float[] jps = new float[] {0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f};
            Matrix4x4 output = psmKinematics.ComputeFk(jps, 7);
            Assert.AreEqual(0.0f, output[0, 0], tol);
            Assert.AreEqual(1.0f, output[0, 1], tol);
            Assert.AreEqual(0.0f, output[0, 2], tol);
            Assert.AreEqual(0.0f, output[0, 3], tol);

            Assert.AreEqual(1.0f, output[1, 0], tol);
            Assert.AreEqual(0.0f, output[1, 1], tol);
            Assert.AreEqual(0.0f, output[1, 2], tol);
            Assert.AreEqual(0.0f, output[1, 3], tol);

            Assert.AreEqual(     0.0f, output[2, 0], tol);
            Assert.AreEqual(     0.0f, output[2, 1], tol);
            Assert.AreEqual(    -1.0f, output[2, 2], tol);
            Assert.AreEqual(-0.10370f, output[2, 3], tol);

            Assert.AreEqual(0.0f, output[3, 0], tol);
            Assert.AreEqual(0.0f, output[3, 1], tol);
            Assert.AreEqual(0.0f, output[3, 2], tol);
            Assert.AreEqual(1.0f, output[3, 3], tol);
        }

        [Test]
        public void TestComputeIkPosQuaternion()
        {
            Quaternion quat = new Quaternion();
            quat.Set(0.7071068f, 0.7071068f, 0.0f, 0.0f);
            Vector3 pos = new Vector3(0.0f, 0.0f, -0.10370f);

            float[] outjps = psmKinematics.ComputeIK(pos, quat);
            float[] correct_jps = new float[] {0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f};
            for(int i = 0 ; i < outjps.Length; i++)
            {
                Assert.AreEqual(
                    correct_jps[i],
                    outjps[i],
                    tol);
            }
        }

        [TearDown]
        public void TearDown()
        {

        }
    }
}
