using System;

using UnityEngine;

[Serializable]
public enum HandPoints : int
{
    Wrist,
    ThumbFirst, //first thumb bone (nearest to the wrist)
    ThumbSecond, //second
    ThumbThird, //third
    ThumbFourth, //fourth
    IndexFingerFirst,
    IndexFingerSecond,
    IndexFingerThird,
    IndexFingerFourth,
    MiddleFingerFirst,
    MiddleFingerSecond,
    MiddleFingerThird,
    MiddleFingerFourth,
    RingFingerFirst,
    RingFingerSecond,
    RingFingerThird,
    RingFingerFourth,
    PinkyFirst, //little finger = pinky
    PinkySecond,
    PinkyThird,
    PinkyFourth,
}

public class HandsPreprocessor : CharacterMapper
{
    private JointPoint[] leftHand;
    private JointPoint[] rightHand;
    private JointPoint[] rightRootFingers;
    private JointPoint[] leftRootFingers;
    private HandPoints[] rootHandPoints;
    private GameObject[] RHandJointsDebug;
    private GameObject[] LHandJointsDebug;


    private JointPoint[] handPoints;

    public CliffConfiguration CliffConfiguration;


    [Header("Anomaly Detection")]
    [SerializeField] private bool enableAnomalyDetector;
    [SerializeField] private HandAnomalyDetector handAnomalyDetector;
    //anomaly denier
    private JointPoint rightElbow;
    private JointPoint leftElbow;
    private Vector3 lastFrameRHandDirection = Vector3.zero;
    private Vector3 lastFrameLHandDirection = Vector3.zero;

    public bool flipHands;

    protected override void InitializationHumanoidPose()
    {

        handPoints = new JointPoint[56];

        for (var i = 0; i < handPoints.Length; i++)
        {
            handPoints[i] = new JointPoint();
        }

        InitializeHand();

        return;

        InitializeRightHand();
        InitializeLeftHand();

        if (debugMode)
        {
            RHandJointsDebug = new GameObject[21];
            LHandJointsDebug = new GameObject[21];
            for (int i = 0; i < RHandJointsDebug.Length; i++)
            {
                RHandJointsDebug[i] = Instantiate(debugGameObject);
            }
            for (int i = 0; i < LHandJointsDebug.Length; i++)
            {
                LHandJointsDebug[i] = Instantiate(debugGameObject);
            }
        }

        rightRootFingers = new JointPoint[5];
        leftRootFingers = new JointPoint[5];
        rootHandPoints = new[]
        {
            HandPoints.ThumbFirst,
            HandPoints.PinkyFirst,
            HandPoints.IndexFingerFirst,
            HandPoints.MiddleFingerFirst,
            HandPoints.RingFingerFirst,
        };

        InitializeRootFingers(rightHand, rightRootFingers);
        InitializeRootFingers(leftHand, leftRootFingers);

        //SetupInverseAndDistance(rightHand);
        //SetupInverseAndDistance(leftHand);

    }


    public void DataCleaner(FrameData[] frameDatas)
    {
        if (enableAnomalyDetector)
            handAnomalyDetector.WrongLandmarkDetector(frameDatas);
    }



    private void PredictHandPose(BodyPartVector[] handLandmarks, JointPoint[] hand)
    {

        for (int i = 0; i < handLandmarks.Length; i++)
        {
            hand[i].LandmarkPose = handLandmarks[i].position;
        }


        // Vector3 forwardFinger = wrist.Transform.position
        //     .TriangleNormal(pinkyFirstLandmark.Transform.position,indexFingerFirst.Transform.position);

        // upward.Normalize();




        /*
        for (int i = 0; i < rootHandPoints.Length; i++)
        {

            JointPoint bone = hand[(int)rootHandPoints[i]];
            float distance = bone.DistanceFromDad;
            Vector3 direction = (-wrist.LandmarkPose + bone.LandmarkPose) / (bone.LandmarkPose - wrist.LandmarkPose).magnitude;

            bone.Transform.position = wrist.Transform.position + direction * distance;

        }
       */

        //setting bone positions
        for (int i = 0; i < hand.Length; i++)
        {

            JointPoint bone = hand[i];

            if (bone.Transform != null) //fourth finger node is not real!
            {
                bone.WorldPos = bone.Transform.position;

            }

        }
        //setting bone positions
        for (int i = 0; i < hand.Length; i++)
        {
            JointPoint bone = hand[i];
            if (bone.Child != null)
            {
                if (bone.Child.Transform != null) //fourth finger node is not real!
                {
                    JointPoint child = bone.Child;
                    float distance = bone.DistanceFromChild;
                    Vector3 direction = (-bone.LandmarkPose + child.LandmarkPose)
                                        / (-bone.LandmarkPose + child.LandmarkPose).magnitude;
                    child.WorldPos = bone.WorldPos + direction * distance;
                    // child.Transform.position = child.WorldPos;

                    continue;
                }
            }
        }


        if (enableKalmanFilter)
            for (int i = 0; i < hand.Length; i++)
            {
                if (hand[i].Transform != null)
                    KalmanUpdate(hand[i]);
            }
        else
        {
            for (int i = 0; i < hand.Length; i++)
            {
                if (hand[i].Transform != null)
                    hand[i].FilteredPos = hand[i].WorldPos;
            }
        }

        // Low pass filter
        if (useLowPassFilter)
        {
            foreach (var jp in hand)
            {
                jp.LastPoses[0] = jp.FilteredPos;
                for (var i = 1; i < jp.LastPoses.Length; i++)
                {
                    jp.LastPoses[i] = jp.LastPoses[i] * lowPassParam + jp.LastPoses[i - 1] * (1f - lowPassParam);
                }
                jp.FilteredPos = jp.LastPoses[jp.LastPoses.Length - 1];
            }
        }

        JointPoint indexFingerFirst = hand[(int)HandPoints.IndexFingerFirst];
        JointPoint wrist = hand[(int)HandPoints.Wrist];
        JointPoint pinkyFirstLandmark = hand[(int)HandPoints.PinkyFirst];
        Vector3 forwardFinger = wrist.FilteredPos
            .TriangleNormal(indexFingerFirst.FilteredPos, pinkyFirstLandmark.FilteredPos);



        //Rotation of the whole hand at First!
        Vector3 normal = wrist.LandmarkPose.TriangleNormal(indexFingerFirst.LandmarkPose, pinkyFirstLandmark.LandmarkPose);
        hand[(int)HandPoints.Wrist].Transform.rotation = Quaternion.LookRotation(-wrist.LandmarkPose + (indexFingerFirst.LandmarkPose + pinkyFirstLandmark.LandmarkPose) / 2.0f, normal) * wrist.InverseRotation;

        //Method2
        // Vector3 normal = wrist.WorldPos.TriangleNormal(indexFingerFirst.WorldPos,pinkyFirstLandmark.WorldPos);
        // hand[(int) HandPoints.Wrist].Transform.rotation = Quaternion.LookRotation(-wrist.WorldPos + (indexFingerFirst.WorldPos + pinkyFirstLandmark.WorldPos)/2.0f, normal) * wrist.InverseRotation;
        //

        //Method3
        // Vector3 normal = wrist.FilteredPos.TriangleNormal(indexFingerFirst.FilteredPos,pinkyFirstLandmark.FilteredPos);
        // hand[(int) HandPoints.Wrist].Transform.rotation = Quaternion.LookRotation(wrist.FilteredPos - (indexFingerFirst.FilteredPos + pinkyFirstLandmark.FilteredPos)/2.0f, normal) * wrist.InverseRotation;
        //


        //rotation

        for (int i = 0; i < hand.Length; i++)
        {

            // if(i == (int) HandPoints.PinkyThird || i == (int) HandPoints.ThumbThird || 
            //    i == (int) HandPoints.IndexFingerThird || i == (int) HandPoints.RingFingerThird
            //    || i == (int) HandPoints.MiddleFingerThird)
            //     continue;
            JointPoint bone = hand[i];

            // if (bone.Parent != null)
            // {
            //     print(bone.Parent.Transform.name);
            //     Vector3 fv = bone.Parent.LandmarkPose - bone.LandmarkPose;
            //     Vector3 forward1 = -bone.LandmarkPose + bone.Child.LandmarkPose;
            //     Vector3 eulur = bone.InitialRotation;
            //     eulur.x += Vector3.Angle(fv, forward1);
            //     bone.Transform.eulerAngles = eulur;
            // }
            // else if (bone.Child != null)
            // {
            //     //forward = hand[(int) HandPoints.Wrist].Transform.position - bone.Transform.position;
            //     //bone.Transform.rotation = Quaternion.LookRotation(bone.LandmarkPose- bone.Child.LandmarkPose, (hand[(int) HandPoints.Wrist].LandmarkPose - bone.LandmarkPose)) * bone.InverseRotation;
            //     // bone.Transform.rotation = Quaternion.LookRotation(bone.LandmarkPose- bone.Child.LandmarkPose, (hand[(int) HandPoints.Wrist].LandmarkPose - bone.LandmarkPose)) * bone.InverseRotation;
            //     //bone.Transform.rotation = Quaternion.LookRotation(bone.LandmarkPose- bone.Child.LandmarkPose, upward) * bone.InverseRotation;
            // }

            if (bone.Parent != null)
            {
                //print(bone.Parent.Transform.name);

                //normalR or forwardFinger this is the problem!

                //Method1
                try
                {
                    //1
                    // Vector3 fv = bone.Parent.FilteredPos - bone.FilteredPos;
                    // bone.Transform.rotation = Quaternion.LookRotation(bone.FilteredPos- bone.Child.FilteredPos, fv) * bone.InverseRotation;
                    //
                    // 1'
                    // bone.Transform.rotation = Quaternion.LookRotation(bone.Transform.position- bone.Child.Transform.position, (forwardFinger + bone.Parent.Transform.right)/2.0f) * bone.InverseRotation;

                    bone.Transform.rotation = Quaternion.LookRotation(bone.FilteredPos - bone.Child.FilteredPos, forwardFinger) * bone.InverseRotation;

                    //2
                    // bone.Transform.rotation = Quaternion.LookRotation(bone.Transform.position- bone.Child.Transform.position, forwardFinger) 
                    //                           * Quaternion.LookRotation(bone.Transform.position- bone.Child.Transform.position, fv)
                    //                           * bone.InverseRotation;

                }
                catch (Exception e)
                {
                    //Method1
                    // Vector3 fv = bone.Parent.LandmarkPose - bone.LandmarkPose;
                    // bone.Transform.rotation = Quaternion.LookRotation(bone.LandmarkPose- bone.Child.LandmarkPose, fv) * bone.InverseRotation;
                    //
                    //Method2
                    //1
                    bone.Transform.rotation = Quaternion.LookRotation(bone.LandmarkPose - bone.Child.LandmarkPose, forwardFinger) * bone.InverseRotation;
                    //2
                    // bone.Transform.rotation = Quaternion.LookRotation(bone.LandmarkPose- bone.Child.LandmarkPose, forwardFinger) 
                    //                           *Quaternion.LookRotation(bone.LandmarkPose- bone.Child.LandmarkPose, fv) 
                    //                           * bone.InverseRotation;

                    // Debug.Log("problem with bone: "+bone.Transform.name);
                }

                //Method2
                // Vector3 fv = bone.Parent.LandmarkPose - bone.LandmarkPose;
                // bone.Transform.rotation = Quaternion.LookRotation(bone.LandmarkPose- bone.Child.LandmarkPose, fv) * bone.InverseRotation;

            }
            else if (bone.Child != null)
            {
                // print(bone.Transform.name);
                //forward = hand[(int) HandPoints.Wrist].Transform.position - bone.Transform.position;
                //Method1
                //bone.Transform.rotation = Quaternion.LookRotation(bone.Transform.position- bone.Child.Transform.position, (bone.Transform.parent.position - bone.Transform.position)) * bone.InverseRotation;
                //Method2
                // bone.Transform.rotation = Quaternion.LookRotation(bone.LandmarkPose- bone.Child.LandmarkPose, (hand[(int) HandPoints.Wrist].LandmarkPose - bone.LandmarkPose)) * bone.InverseRotation;
                //bone.Transform.rotation = Quaternion.LookRotation(bone.LandmarkPose- bone.Child.LandmarkPose, upward) * bone.InverseRotation;
                //Method3
                // bone.Transform.rotation = Quaternion.LookRotation(bone.Transform.position- bone.Child.Transform.position, (wrist.Transform.position - bone.Transform.position)) * bone.InverseRotation;
                //Method4
                bone.Transform.rotation = Quaternion.LookRotation(bone.FilteredPos - bone.Child.FilteredPos, forwardFinger) * bone.InverseRotation;

            }
            /*
            if (bone.Parent != null)
            {
                Vector3 fv = bone.Parent.Transform.position - bone.Transform.position;
                bone.Transform.rotation = Quaternion.LookRotation(bone.Transform.position- bone.Child.Transform.position, fv) * bone.InverseRotation;
            }
            
            else if (bone.Child != null)
            {
                bone.Transform.rotation = Quaternion.LookRotation(bone.Transform.position- bone.Child.Transform.position, forward) * bone.InverseRotation;
            }
            */
        }

        //Rotation of the whole hand at the end!
        // Vector3 normal = wrist.LandmarkPose.TriangleNormal(indexFingerFirst.LandmarkPose,pinkyFirstLandmark.LandmarkPose);
        // hand[(int) HandPoints.Wrist].Transform.rotation = Quaternion.LookRotation(-wrist.LandmarkPose + (indexFingerFirst.LandmarkPose + pinkyFirstLandmark.LandmarkPose)/2.0f, normal) * wrist.InverseRotation;

        //Method2
        // Vector3 normal = wrist.Transform.position.TriangleNormal(indexFingerFirst.Transform.position,pinkyFirstLandmark.Transform.position);
        // hand[(int) HandPoints.Wrist].Transform.rotation = Quaternion.LookRotation(-wrist.Transform.position + (indexFingerFirst.Transform.position + pinkyFirstLandmark.Transform.position)/2.0f, normal) * wrist.InverseRotation;
        //     

        //Method3
        // Vector3 normal = wrist.FilteredPos.TriangleNormal(indexFingerFirst.FilteredPos,pinkyFirstLandmark.FilteredPos);
        // hand[(int) HandPoints.Wrist].Transform.rotation = Quaternion.LookRotation(-wrist.FilteredPos + (indexFingerFirst.FilteredPos + pinkyFirstLandmark.FilteredPos)/2.0f, normal) * wrist.InverseRotation;
        //

    }

    public void Predict3DPose(HandJsonVector poseJsonVector)
    {
        try
        {
            //right hand
            BodyPartVector[] handR = poseJsonVector.handsR;
            BodyPartVector[] handL = poseJsonVector.handsL;
            if (flipHands)
            {
                handR = poseJsonVector.handsL;
                handL = poseJsonVector.handsR;
            }
            if (handR != null)
                if (handR.Length != 0)
                {
                    int rHandStat = RightHandAnomalyDetector(handR);
                    if (rHandStat == 0)
                    {
                        PredictHandPose(handR, rightHand);
                    }
                    else if (rHandStat != 2)
                    {
                        if (handL != null)
                            if (handL.Length != 0)
                                PredictHandPose(handL, rightHand);
                        Debug.Log("Anomaly detected! R");
                    }
                    if (debugMode)
                        for (int i = 0; i < handR.Length; i++)
                        {
                            RHandJointsDebug[i].transform.position = handR[i].position;
                        }
                }

            if (handL != null)
                if (handL.Length != 0)
                {
                    int lHandStat = LeftHandAnomalyDetector(handL);
                    if (lHandStat == 0)
                    {
                        PredictHandPose(handL, leftHand);
                    }
                    else if (lHandStat != 2)
                    {
                        if (handR != null)
                            if (handR.Length != 0)
                                PredictHandPose(handR, leftHand);
                        Debug.Log("Anomaly detected! L");
                    }
                    if (debugMode)
                        for (int i = 0; i < handL.Length; i++)
                        {
                            LHandJointsDebug[i].transform.position = handL[i].position;
                        }
                }

        }
        catch (Exception e)
        {
            Debug.LogError("Hand Problem!");

            Console.WriteLine(e);
            throw;
        }
    }


    //Anomaly detectors
    // 0 means its all good
    // 1 means should flip hands
    // 2 means should not calculate current frame
    private int RightHandAnomalyDetector(BodyPartVector[] handLandmarks)
    {
        return 0;
        Vector3 rightArmVector = rightHand[(int)HandPoints.Wrist].Transform.position - rightElbow.Transform.position;
        Vector3 indexFingerFirst = handLandmarks[(int)HandPoints.IndexFingerFirst].position;
        Vector3 wrist = handLandmarks[(int)HandPoints.Wrist].position;
        Vector3 pinkyFirstLandmark = handLandmarks[(int)HandPoints.PinkyFirst].position;
        Vector3 handVector = -wrist + (indexFingerFirst + pinkyFirstLandmark) / 2.0f;

        Vector3 newHandDirection = wrist
            .TriangleNormal(indexFingerFirst, pinkyFirstLandmark);
        if (!lastFrameRHandDirection.Equals(Vector3.zero))
        {
            if (Vector3.Angle(newHandDirection, lastFrameRHandDirection) > 100 ||
                Vector3.Angle(newHandDirection, lastFrameRHandDirection) < -100)
            {
                Debug.Log("1 frame changed and more than 100 degree rotation!");
                // lastFrameRHandDirection = newHandDirection;
                return 2;
            }
        }
        lastFrameRHandDirection = newHandDirection;
        if (Vector3.Angle(rightArmVector, handVector) > 100 || Vector3.Angle(rightArmVector, handVector) < -100)
        {
            Debug.Log(Vector3.Angle(rightArmVector, handVector));

            return 1;
        }

        return 0;
    }
    private int LeftHandAnomalyDetector(BodyPartVector[] handLandmarks)
    {
        return 0;
        Vector3 leftArmVector = leftHand[(int)HandPoints.Wrist].Transform.position - leftElbow.Transform.position;
        Vector3 indexFingerFirst = handLandmarks[(int)HandPoints.IndexFingerFirst].position;
        Vector3 wrist = handLandmarks[(int)HandPoints.Wrist].position;
        Vector3 pinkyFirstLandmark = handLandmarks[(int)HandPoints.PinkyFirst].position;
        Vector3 handVector = -wrist + (indexFingerFirst + pinkyFirstLandmark) / 2.0f;

        Vector3 newHandDirection = wrist
            .TriangleNormal(indexFingerFirst, pinkyFirstLandmark);


        if (!lastFrameLHandDirection.Equals(Vector3.zero))
        {
            if (Vector3.Angle(newHandDirection, lastFrameLHandDirection) > 100 ||
                Vector3.Angle(newHandDirection, lastFrameLHandDirection) < -100)
            {
                Debug.Log("1 frame changed and more than 100 degree rotation!");
                // lastFrameRHandDirection = newHandDirection;
                return 2;
            }
        }
        lastFrameLHandDirection = newHandDirection;

        if (Vector3.Angle(leftArmVector, handVector) > 100 || Vector3.Angle(leftArmVector, handVector) < -100)
        {
            Debug.Log(Vector3.Angle(leftArmVector, handVector));
            return 1;
        }
        return 0;
    }

    public void ClearFramesCache()
    {
        lastFrameRHandDirection = Vector3.zero;
        lastFrameLHandDirection = Vector3.zero;
    }

    private void InitializeRootFingers(JointPoint[] bones, JointPoint[] rootFingers)
    {
        rootFingers[0] = new JointPoint()
        {
            Transform = bones[(int)HandPoints.ThumbFirst].Transform
        };
        rootFingers[1] = new JointPoint()
        {
            Transform = bones[(int)HandPoints.PinkyFirst].Transform
        };
        rootFingers[2] = new JointPoint()
        {
            Transform = bones[(int)HandPoints.MiddleFingerFirst].Transform
        };
        rootFingers[3] = new JointPoint()
        {
            Transform = bones[(int)HandPoints.RingFingerFirst].Transform
        };
        rootFingers[4] = new JointPoint()
        {
            Transform = bones[(int)HandPoints.IndexFingerFirst].Transform
        };
        for (int i = 0; i < rootFingers.Length; i++)
        {
            rootFingers[i].Parent = bones[(int)HandPoints.Wrist];
        }
    }

    private void InitializeHand()
    {
        //handPoints = new JointPoint[56];

        #region Right Hand

        //////RightThumbProximal

        handPoints[(int)HumanBodyBones.RightThumbProximal].Transform = anim.GetBoneTransform(HumanBodyBones.RightThumbProximal);
        handPoints[(int)HumanBodyBones.RightThumbProximal].DefaultRotation = handPoints[(int)HumanBodyBones.RightThumbProximal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.RightThumbProximal].InitRotation = handPoints[(int)HumanBodyBones.RightThumbProximal].Transform.rotation;

        //////RightThumbIntermediate

        handPoints[(int)HumanBodyBones.RightThumbIntermediate].Transform = anim.GetBoneTransform(HumanBodyBones.RightThumbIntermediate);
        handPoints[(int)HumanBodyBones.RightThumbIntermediate].DefaultRotation = handPoints[(int)HumanBodyBones.RightThumbIntermediate].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.RightThumbIntermediate].InitRotation = handPoints[(int)HumanBodyBones.RightThumbIntermediate].Transform.rotation;

        //////RightThumbIntermediate

        handPoints[(int)HumanBodyBones.RightThumbDistal].Transform = anim.GetBoneTransform(HumanBodyBones.RightThumbDistal);
        handPoints[(int)HumanBodyBones.RightThumbDistal].DefaultRotation = handPoints[(int)HumanBodyBones.RightThumbDistal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.RightThumbDistal].InitRotation = handPoints[(int)HumanBodyBones.RightThumbDistal].Transform.rotation;





        //////RightIndexProximal

        handPoints[(int)HumanBodyBones.RightIndexProximal].Transform = anim.GetBoneTransform(HumanBodyBones.RightIndexProximal);
        handPoints[(int)HumanBodyBones.RightIndexProximal].DefaultRotation = handPoints[(int)HumanBodyBones.RightIndexProximal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.RightIndexProximal].InitRotation = handPoints[(int)HumanBodyBones.RightIndexProximal].Transform.rotation;

        //////RightIndexIntermediate

        handPoints[(int)HumanBodyBones.RightIndexIntermediate].Transform = anim.GetBoneTransform(HumanBodyBones.RightIndexIntermediate);
        handPoints[(int)HumanBodyBones.RightIndexIntermediate].DefaultRotation = handPoints[(int)HumanBodyBones.RightIndexIntermediate].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.RightIndexIntermediate].InitRotation = handPoints[(int)HumanBodyBones.RightIndexIntermediate].Transform.rotation;

        //////RightIndexDistal

        handPoints[(int)HumanBodyBones.RightIndexDistal].Transform = anim.GetBoneTransform(HumanBodyBones.RightIndexDistal);
        handPoints[(int)HumanBodyBones.RightIndexDistal].DefaultRotation = handPoints[(int)HumanBodyBones.RightIndexDistal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.RightIndexDistal].InitRotation = handPoints[(int)HumanBodyBones.RightIndexDistal].Transform.rotation;







        //////RightMiddleProximal

        handPoints[(int)HumanBodyBones.RightMiddleProximal].Transform = anim.GetBoneTransform(HumanBodyBones.RightMiddleProximal);
        handPoints[(int)HumanBodyBones.RightMiddleProximal].DefaultRotation = handPoints[(int)HumanBodyBones.RightMiddleProximal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.RightMiddleProximal].InitRotation = handPoints[(int)HumanBodyBones.RightMiddleProximal].Transform.rotation;

        //////RightMiddleIntermediate

        handPoints[(int)HumanBodyBones.RightMiddleIntermediate].Transform = anim.GetBoneTransform(HumanBodyBones.RightMiddleIntermediate);
        handPoints[(int)HumanBodyBones.RightMiddleIntermediate].DefaultRotation = handPoints[(int)HumanBodyBones.RightMiddleIntermediate].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.RightMiddleIntermediate].InitRotation = handPoints[(int)HumanBodyBones.RightMiddleIntermediate].Transform.rotation;

        //////RightMiddleDistal

        handPoints[(int)HumanBodyBones.RightMiddleDistal].Transform = anim.GetBoneTransform(HumanBodyBones.RightMiddleDistal);
        handPoints[(int)HumanBodyBones.RightMiddleDistal].DefaultRotation = handPoints[(int)HumanBodyBones.RightMiddleDistal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.RightMiddleDistal].InitRotation = handPoints[(int)HumanBodyBones.RightMiddleDistal].Transform.rotation;







        //////RightRingProximal

        handPoints[(int)HumanBodyBones.RightRingProximal].Transform = anim.GetBoneTransform(HumanBodyBones.RightRingProximal);
        handPoints[(int)HumanBodyBones.RightRingProximal].DefaultRotation = handPoints[(int)HumanBodyBones.RightRingProximal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.RightRingProximal].InitRotation = handPoints[(int)HumanBodyBones.RightRingProximal].Transform.rotation;

        //////RightRingIntermediate

        handPoints[(int)HumanBodyBones.RightRingIntermediate].Transform = anim.GetBoneTransform(HumanBodyBones.RightRingIntermediate);
        handPoints[(int)HumanBodyBones.RightRingIntermediate].DefaultRotation = handPoints[(int)HumanBodyBones.RightRingIntermediate].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.RightRingIntermediate].InitRotation = handPoints[(int)HumanBodyBones.RightRingIntermediate].Transform.rotation;

        //////RightRingDistal

        handPoints[(int)HumanBodyBones.RightRingDistal].Transform = anim.GetBoneTransform(HumanBodyBones.RightRingDistal);
        handPoints[(int)HumanBodyBones.RightRingDistal].DefaultRotation = handPoints[(int)HumanBodyBones.RightRingDistal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.RightRingDistal].InitRotation = handPoints[(int)HumanBodyBones.RightRingDistal].Transform.rotation;








        //////RightLittleProximal

        handPoints[(int)HumanBodyBones.RightLittleProximal].Transform = anim.GetBoneTransform(HumanBodyBones.RightLittleProximal);
        handPoints[(int)HumanBodyBones.RightLittleProximal].DefaultRotation = handPoints[(int)HumanBodyBones.RightLittleProximal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.RightLittleProximal].InitRotation = handPoints[(int)HumanBodyBones.RightLittleProximal].Transform.rotation;

        //////RightLittleIntermediate

        handPoints[(int)HumanBodyBones.RightLittleIntermediate].Transform = anim.GetBoneTransform(HumanBodyBones.RightLittleIntermediate);
        handPoints[(int)HumanBodyBones.RightLittleIntermediate].DefaultRotation = handPoints[(int)HumanBodyBones.RightLittleIntermediate].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.RightLittleIntermediate].InitRotation = handPoints[(int)HumanBodyBones.RightLittleIntermediate].Transform.rotation;

        //////RightLittleDistal

        handPoints[(int)HumanBodyBones.RightLittleDistal].Transform = anim.GetBoneTransform(HumanBodyBones.RightLittleDistal);
        handPoints[(int)HumanBodyBones.RightLittleDistal].DefaultRotation = handPoints[(int)HumanBodyBones.RightLittleDistal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.RightLittleDistal].InitRotation = handPoints[(int)HumanBodyBones.RightLittleDistal].Transform.rotation;

        #endregion



        #region Left Hand

        //////RightThumbProximal

        handPoints[(int)HumanBodyBones.LeftThumbProximal].Transform = anim.GetBoneTransform(HumanBodyBones.LeftThumbProximal);
        handPoints[(int)HumanBodyBones.LeftThumbProximal].DefaultRotation = handPoints[(int)HumanBodyBones.LeftThumbProximal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.LeftThumbProximal].InitRotation = handPoints[(int)HumanBodyBones.LeftThumbProximal].Transform.rotation;

        //////LeftThumbIntermediate

        handPoints[(int)HumanBodyBones.LeftThumbIntermediate].Transform = anim.GetBoneTransform(HumanBodyBones.LeftThumbIntermediate);
        handPoints[(int)HumanBodyBones.LeftThumbIntermediate].DefaultRotation = handPoints[(int)HumanBodyBones.LeftThumbIntermediate].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.LeftThumbIntermediate].InitRotation = handPoints[(int)HumanBodyBones.LeftThumbIntermediate].Transform.rotation;

        //////LeftThumbIntermediate

        handPoints[(int)HumanBodyBones.LeftThumbDistal].Transform = anim.GetBoneTransform(HumanBodyBones.LeftThumbDistal);
        handPoints[(int)HumanBodyBones.LeftThumbDistal].DefaultRotation = handPoints[(int)HumanBodyBones.LeftThumbDistal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.LeftThumbDistal].InitRotation = handPoints[(int)HumanBodyBones.LeftThumbDistal].Transform.rotation;





        //////LeftIndexProximal

        handPoints[(int)HumanBodyBones.LeftIndexProximal].Transform = anim.GetBoneTransform(HumanBodyBones.LeftIndexProximal);
        handPoints[(int)HumanBodyBones.LeftIndexProximal].DefaultRotation = handPoints[(int)HumanBodyBones.LeftIndexProximal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.LeftIndexProximal].InitRotation = handPoints[(int)HumanBodyBones.LeftIndexProximal].Transform.rotation;

        //////LeftIndexIntermediate

        handPoints[(int)HumanBodyBones.LeftIndexIntermediate].Transform = anim.GetBoneTransform(HumanBodyBones.LeftIndexIntermediate);
        handPoints[(int)HumanBodyBones.LeftIndexIntermediate].DefaultRotation = handPoints[(int)HumanBodyBones.LeftIndexIntermediate].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.LeftIndexIntermediate].InitRotation = handPoints[(int)HumanBodyBones.LeftIndexIntermediate].Transform.rotation;

        //////LeftIndexDistal

        handPoints[(int)HumanBodyBones.LeftIndexDistal].Transform = anim.GetBoneTransform(HumanBodyBones.LeftIndexDistal);
        handPoints[(int)HumanBodyBones.LeftIndexDistal].DefaultRotation = handPoints[(int)HumanBodyBones.LeftIndexDistal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.LeftIndexDistal].InitRotation = handPoints[(int)HumanBodyBones.LeftIndexDistal].Transform.rotation;







        //////LeftMiddleProximal

        handPoints[(int)HumanBodyBones.LeftMiddleProximal].Transform = anim.GetBoneTransform(HumanBodyBones.LeftMiddleProximal);
        handPoints[(int)HumanBodyBones.LeftMiddleProximal].DefaultRotation = handPoints[(int)HumanBodyBones.LeftMiddleProximal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.LeftMiddleProximal].InitRotation = handPoints[(int)HumanBodyBones.LeftMiddleProximal].Transform.rotation;

        //////LeftMiddleIntermediate

        handPoints[(int)HumanBodyBones.LeftMiddleIntermediate].Transform = anim.GetBoneTransform(HumanBodyBones.LeftMiddleIntermediate);
        handPoints[(int)HumanBodyBones.LeftMiddleIntermediate].DefaultRotation = handPoints[(int)HumanBodyBones.LeftMiddleIntermediate].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.LeftMiddleIntermediate].InitRotation = handPoints[(int)HumanBodyBones.LeftMiddleIntermediate].Transform.rotation;

        //////LeftMiddleDistal

        handPoints[(int)HumanBodyBones.LeftMiddleDistal].Transform = anim.GetBoneTransform(HumanBodyBones.LeftMiddleDistal);
        handPoints[(int)HumanBodyBones.LeftMiddleDistal].DefaultRotation = handPoints[(int)HumanBodyBones.LeftMiddleDistal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.LeftMiddleDistal].InitRotation = handPoints[(int)HumanBodyBones.LeftMiddleDistal].Transform.rotation;







        //////LeftRingProximal

        handPoints[(int)HumanBodyBones.LeftRingProximal].Transform = anim.GetBoneTransform(HumanBodyBones.LeftRingProximal);
        handPoints[(int)HumanBodyBones.LeftRingProximal].DefaultRotation = handPoints[(int)HumanBodyBones.LeftRingProximal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.LeftRingProximal].InitRotation = handPoints[(int)HumanBodyBones.LeftRingProximal].Transform.rotation;

        //////LeftRingIntermediate

        handPoints[(int)HumanBodyBones.LeftRingIntermediate].Transform = anim.GetBoneTransform(HumanBodyBones.LeftRingIntermediate);
        handPoints[(int)HumanBodyBones.LeftRingIntermediate].DefaultRotation = handPoints[(int)HumanBodyBones.LeftRingIntermediate].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.LeftRingIntermediate].InitRotation = handPoints[(int)HumanBodyBones.LeftRingIntermediate].Transform.rotation;

        //////LeftRingDistal

        handPoints[(int)HumanBodyBones.LeftRingDistal].Transform = anim.GetBoneTransform(HumanBodyBones.LeftRingDistal);
        handPoints[(int)HumanBodyBones.LeftRingDistal].DefaultRotation = handPoints[(int)HumanBodyBones.LeftRingDistal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.LeftRingDistal].InitRotation = handPoints[(int)HumanBodyBones.LeftRingDistal].Transform.rotation;








        //////LeftLittleProximal

        handPoints[(int)HumanBodyBones.LeftLittleProximal].Transform = anim.GetBoneTransform(HumanBodyBones.LeftLittleProximal);
        handPoints[(int)HumanBodyBones.LeftLittleProximal].DefaultRotation = handPoints[(int)HumanBodyBones.LeftLittleProximal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.LeftLittleProximal].InitRotation = handPoints[(int)HumanBodyBones.LeftLittleProximal].Transform.rotation;

        //////LeftLittleIntermediate

        handPoints[(int)HumanBodyBones.LeftLittleIntermediate].Transform = anim.GetBoneTransform(HumanBodyBones.LeftLittleIntermediate);
        handPoints[(int)HumanBodyBones.LeftLittleIntermediate].DefaultRotation = handPoints[(int)HumanBodyBones.LeftLittleIntermediate].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.LeftLittleIntermediate].InitRotation = handPoints[(int)HumanBodyBones.LeftLittleIntermediate].Transform.rotation;

        //////LeftLittleDistal

        handPoints[(int)HumanBodyBones.LeftLittleDistal].Transform = anim.GetBoneTransform(HumanBodyBones.LeftLittleDistal);
        handPoints[(int)HumanBodyBones.LeftLittleDistal].DefaultRotation = handPoints[(int)HumanBodyBones.LeftLittleDistal].Transform.localEulerAngles;
        handPoints[(int)HumanBodyBones.LeftLittleDistal].InitRotation = handPoints[(int)HumanBodyBones.LeftLittleDistal].Transform.rotation;

        #endregion

    }

    private void InitializeRightHand()
    {
        // Right Hand
        rightHand = new JointPoint[21];

        for (var i = 0; i < rightHand.Length; i++)
        {
            rightHand[i] = new JointPoint();
            rightHand[i].LastPoses = new Vector3[lowPassFilterChannels];

        }

        //Wrist
        rightHand[(int)HandPoints.Wrist].Transform = anim.GetBoneTransform(HumanBodyBones.RightHand);

        //thumb
        rightHand[(int)HandPoints.ThumbFirst].Transform = anim.GetBoneTransform(HumanBodyBones.RightThumbProximal);
        rightHand[(int)HandPoints.ThumbSecond].Transform = anim.GetBoneTransform(HumanBodyBones.RightThumbIntermediate);
        rightHand[(int)HandPoints.ThumbThird].Transform = anim.GetBoneTransform(HumanBodyBones.RightThumbDistal);

        //child and parent
        rightHand[(int)HandPoints.ThumbFirst].Child = rightHand[(int)HandPoints.ThumbSecond];
        rightHand[(int)HandPoints.ThumbSecond].Child = rightHand[(int)HandPoints.ThumbThird];
        rightHand[(int)HandPoints.ThumbThird].Child = rightHand[(int)HandPoints.ThumbFourth];
        rightHand[(int)HandPoints.ThumbSecond].Parent = rightHand[(int)HandPoints.ThumbFirst];
        rightHand[(int)HandPoints.ThumbThird].Parent = rightHand[(int)HandPoints.ThumbSecond];

        //index
        rightHand[(int)HandPoints.IndexFingerFirst].Transform = anim.GetBoneTransform(HumanBodyBones.RightIndexProximal);
        rightHand[(int)HandPoints.IndexFingerSecond].Transform = anim.GetBoneTransform(HumanBodyBones.RightIndexIntermediate);
        rightHand[(int)HandPoints.IndexFingerThird].Transform = anim.GetBoneTransform(HumanBodyBones.RightIndexDistal);
        //child and parent
        rightHand[(int)HandPoints.IndexFingerFirst].Child = rightHand[(int)HandPoints.IndexFingerSecond];
        rightHand[(int)HandPoints.IndexFingerSecond].Child = rightHand[(int)HandPoints.IndexFingerThird];
        rightHand[(int)HandPoints.IndexFingerThird].Child = rightHand[(int)HandPoints.IndexFingerFourth];
        rightHand[(int)HandPoints.IndexFingerSecond].Parent = rightHand[(int)HandPoints.IndexFingerFirst];
        rightHand[(int)HandPoints.IndexFingerThird].Parent = rightHand[(int)HandPoints.IndexFingerSecond];

        //middle
        rightHand[(int)HandPoints.MiddleFingerFirst].Transform = anim.GetBoneTransform(HumanBodyBones.RightMiddleProximal);
        rightHand[(int)HandPoints.MiddleFingerSecond].Transform = anim.GetBoneTransform(HumanBodyBones.RightMiddleIntermediate);
        rightHand[(int)HandPoints.MiddleFingerThird].Transform = anim.GetBoneTransform(HumanBodyBones.RightMiddleDistal);
        //child and parent
        rightHand[(int)HandPoints.MiddleFingerFirst].Child = rightHand[(int)HandPoints.MiddleFingerSecond];
        rightHand[(int)HandPoints.MiddleFingerSecond].Child = rightHand[(int)HandPoints.MiddleFingerThird];
        rightHand[(int)HandPoints.MiddleFingerThird].Child = rightHand[(int)HandPoints.MiddleFingerFourth];
        rightHand[(int)HandPoints.MiddleFingerSecond].Parent = rightHand[(int)HandPoints.MiddleFingerFirst];
        rightHand[(int)HandPoints.MiddleFingerThird].Parent = rightHand[(int)HandPoints.MiddleFingerSecond];

        //ring
        rightHand[(int)HandPoints.RingFingerFirst].Transform = anim.GetBoneTransform(HumanBodyBones.RightRingProximal);
        rightHand[(int)HandPoints.RingFingerSecond].Transform = anim.GetBoneTransform(HumanBodyBones.RightRingIntermediate);
        rightHand[(int)HandPoints.RingFingerThird].Transform = anim.GetBoneTransform(HumanBodyBones.RightRingDistal);
        //child and parent
        rightHand[(int)HandPoints.RingFingerFirst].Child = rightHand[(int)HandPoints.RingFingerSecond];
        rightHand[(int)HandPoints.RingFingerSecond].Child = rightHand[(int)HandPoints.RingFingerThird];
        rightHand[(int)HandPoints.RingFingerThird].Child = rightHand[(int)HandPoints.RingFingerFourth];
        rightHand[(int)HandPoints.RingFingerSecond].Parent = rightHand[(int)HandPoints.RingFingerFirst];
        rightHand[(int)HandPoints.RingFingerThird].Parent = rightHand[(int)HandPoints.RingFingerSecond];

        //pinky
        rightHand[(int)HandPoints.PinkyFirst].Transform = anim.GetBoneTransform(HumanBodyBones.RightLittleProximal);
        rightHand[(int)HandPoints.PinkySecond].Transform = anim.GetBoneTransform(HumanBodyBones.RightLittleIntermediate);
        rightHand[(int)HandPoints.PinkyThird].Transform = anim.GetBoneTransform(HumanBodyBones.RightLittleDistal);
        //child and parent
        rightHand[(int)HandPoints.PinkyFirst].Child = rightHand[(int)HandPoints.PinkySecond];
        rightHand[(int)HandPoints.PinkySecond].Child = rightHand[(int)HandPoints.PinkyThird];
        rightHand[(int)HandPoints.PinkyThird].Child = rightHand[(int)HandPoints.PinkyFourth];
        rightHand[(int)HandPoints.PinkySecond].Parent = rightHand[(int)HandPoints.PinkyFirst];
        rightHand[(int)HandPoints.PinkyThird].Parent = rightHand[(int)HandPoints.PinkySecond];

        //elbow
        rightElbow = new JointPoint();
        rightElbow.Transform = anim.GetBoneTransform(HumanBodyBones.RightLowerArm);
    }

    private void InitializeLeftHand()
    {






        // Right Hand
        leftHand = new JointPoint[21];

        for (var i = 0; i < leftHand.Length; i++)
        {
            leftHand[i] = new JointPoint();
            leftHand[i].LastPoses = new Vector3[lowPassFilterChannels];

        }

        //Wrist
        leftHand[(int)HandPoints.Wrist].Transform = anim.GetBoneTransform(HumanBodyBones.LeftHand);

        //thumb
        leftHand[(int)HandPoints.ThumbFirst].Transform = anim.GetBoneTransform(HumanBodyBones.LeftThumbProximal);
        leftHand[(int)HandPoints.ThumbSecond].Transform = anim.GetBoneTransform(HumanBodyBones.LeftThumbIntermediate);
        leftHand[(int)HandPoints.ThumbThird].Transform = anim.GetBoneTransform(HumanBodyBones.LeftThumbDistal);
        //child and parent
        leftHand[(int)HandPoints.ThumbFirst].Child = leftHand[(int)HandPoints.ThumbSecond];
        leftHand[(int)HandPoints.ThumbSecond].Child = leftHand[(int)HandPoints.ThumbThird];
        leftHand[(int)HandPoints.ThumbThird].Child = leftHand[(int)HandPoints.ThumbFourth];
        leftHand[(int)HandPoints.ThumbSecond].Parent = leftHand[(int)HandPoints.ThumbFirst];
        leftHand[(int)HandPoints.ThumbThird].Parent = leftHand[(int)HandPoints.ThumbSecond];

        //index
        leftHand[(int)HandPoints.IndexFingerFirst].Transform = anim.GetBoneTransform(HumanBodyBones.LeftIndexProximal);
        leftHand[(int)HandPoints.IndexFingerSecond].Transform = anim.GetBoneTransform(HumanBodyBones.LeftIndexIntermediate);
        leftHand[(int)HandPoints.IndexFingerThird].Transform = anim.GetBoneTransform(HumanBodyBones.LeftIndexDistal);
        //child and parent
        leftHand[(int)HandPoints.IndexFingerFirst].Child = leftHand[(int)HandPoints.IndexFingerSecond];
        leftHand[(int)HandPoints.IndexFingerSecond].Child = leftHand[(int)HandPoints.IndexFingerThird];
        leftHand[(int)HandPoints.IndexFingerThird].Child = leftHand[(int)HandPoints.IndexFingerFourth];
        leftHand[(int)HandPoints.IndexFingerSecond].Parent = leftHand[(int)HandPoints.IndexFingerFirst];
        leftHand[(int)HandPoints.IndexFingerThird].Parent = leftHand[(int)HandPoints.IndexFingerSecond];

        //middle
        leftHand[(int)HandPoints.MiddleFingerFirst].Transform = anim.GetBoneTransform(HumanBodyBones.LeftMiddleProximal);
        leftHand[(int)HandPoints.MiddleFingerSecond].Transform = anim.GetBoneTransform(HumanBodyBones.LeftMiddleIntermediate);
        leftHand[(int)HandPoints.MiddleFingerThird].Transform = anim.GetBoneTransform(HumanBodyBones.LeftMiddleDistal);
        //child and parent
        leftHand[(int)HandPoints.MiddleFingerFirst].Child = leftHand[(int)HandPoints.MiddleFingerSecond];
        leftHand[(int)HandPoints.MiddleFingerSecond].Child = leftHand[(int)HandPoints.MiddleFingerThird];
        leftHand[(int)HandPoints.MiddleFingerThird].Child = leftHand[(int)HandPoints.MiddleFingerFourth];
        leftHand[(int)HandPoints.MiddleFingerSecond].Parent = leftHand[(int)HandPoints.MiddleFingerFirst];
        leftHand[(int)HandPoints.MiddleFingerThird].Parent = leftHand[(int)HandPoints.MiddleFingerSecond];

        //ring
        leftHand[(int)HandPoints.RingFingerFirst].Transform = anim.GetBoneTransform(HumanBodyBones.LeftRingProximal);
        leftHand[(int)HandPoints.RingFingerSecond].Transform = anim.GetBoneTransform(HumanBodyBones.LeftRingIntermediate);
        leftHand[(int)HandPoints.RingFingerThird].Transform = anim.GetBoneTransform(HumanBodyBones.LeftRingDistal);
        //child and parent
        leftHand[(int)HandPoints.RingFingerFirst].Child = leftHand[(int)HandPoints.RingFingerSecond];
        leftHand[(int)HandPoints.RingFingerSecond].Child = leftHand[(int)HandPoints.RingFingerThird];
        leftHand[(int)HandPoints.RingFingerThird].Child = leftHand[(int)HandPoints.RingFingerFourth];
        leftHand[(int)HandPoints.RingFingerSecond].Parent = leftHand[(int)HandPoints.RingFingerFirst];
        leftHand[(int)HandPoints.RingFingerThird].Parent = leftHand[(int)HandPoints.RingFingerSecond];

        //pinky
        leftHand[(int)HandPoints.PinkyFirst].Transform = anim.GetBoneTransform(HumanBodyBones.LeftLittleProximal);
        leftHand[(int)HandPoints.PinkySecond].Transform = anim.GetBoneTransform(HumanBodyBones.LeftLittleIntermediate);
        leftHand[(int)HandPoints.PinkyThird].Transform = anim.GetBoneTransform(HumanBodyBones.LeftLittleDistal);
        //child and parent
        leftHand[(int)HandPoints.PinkyFirst].Child = leftHand[(int)HandPoints.PinkySecond];
        leftHand[(int)HandPoints.PinkySecond].Child = leftHand[(int)HandPoints.PinkyThird];
        leftHand[(int)HandPoints.PinkyThird].Child = leftHand[(int)HandPoints.PinkyFourth];
        leftHand[(int)HandPoints.PinkySecond].Parent = leftHand[(int)HandPoints.PinkyFirst];
        leftHand[(int)HandPoints.PinkyThird].Parent = leftHand[(int)HandPoints.PinkySecond];

        //elbow
        leftElbow = new JointPoint();
        leftElbow.Transform = anim.GetBoneTransform(HumanBodyBones.LeftLowerArm);
    }



    #region Cliff

    public void PredictCliff3DPose(CliffFrame cliffFrame)
    {

        bool isNew = false;
        bool isQuaternation = false;

        bool isGlobal = CliffConfiguration.isGlobal;
        bool applyDifference = CliffConfiguration.applyDifference;

        #region MyRegion
        // demo_data_unity_quat_TPoseVideo Test
        // 1 good
        // 4 bad
        // 2 3 terrible

        // demo_data_unity_rota_TPoseVideo Test
        // 1 good
        // 4 bad
        // 2 3 terrible

        // demo_data_unity_quat_wangxi Test
        // 1 good
        // 4 bad
        // 2 3 terrible 
        #endregion

        switch ((int)CliffConfiguration.CliffStatee)
        {
            case 1:
                isNew = true;
                isQuaternation = false;
                break;

            case 2:
                isNew = true;
                isQuaternation = true;
                break;

            case 3:
                isNew = false;
                isQuaternation = true;
                break;

            case 4:
                isNew = false;
                isQuaternation = false;
                break;

            default:
                break;
        }



        #region Right Hand



        // RightThumbProximal
        var RightThumbProximal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightHandThumb1];
        CalculateRotation(handPoints[(int)HumanBodyBones.RightThumbProximal], RightThumbProximal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // RightThumbProximal
        var RightThumbIntermediate = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightHandThumb2];
        CalculateRotation(handPoints[(int)HumanBodyBones.RightThumbIntermediate], RightThumbIntermediate, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // RightThumbProximal
        var RightThumbDistal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightHandThumb3];
        CalculateRotation(handPoints[(int)HumanBodyBones.RightThumbDistal], RightThumbDistal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);








        // RightIndexProximal
        var RightIndexProximal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightHandIndex1];
        CalculateRotation(handPoints[(int)HumanBodyBones.RightIndexProximal], RightIndexProximal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // RightIndexIntermediate
        var RightIndexIntermediate = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightHandIndex2];
        CalculateRotation(handPoints[(int)HumanBodyBones.RightIndexIntermediate], RightIndexIntermediate, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // RightIndexDistal
        var RightIndexDistal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightHandIndex3];
        CalculateRotation(handPoints[(int)HumanBodyBones.RightIndexDistal], RightIndexDistal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);













        // RightMiddleProximal
        var RightMiddleProximal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightHandMiddle1];
        CalculateRotation(handPoints[(int)HumanBodyBones.RightMiddleProximal], RightMiddleProximal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // RightMiddleIntermediate
        var RightMiddleIntermediate = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightHandMiddle2];
        CalculateRotation(handPoints[(int)HumanBodyBones.RightMiddleIntermediate], RightMiddleIntermediate, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // RightMiddleDistal
        var RightMiddleDistal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightHandMiddle3];
        CalculateRotation(handPoints[(int)HumanBodyBones.RightMiddleDistal], RightMiddleDistal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);













        // RightRingProximal
        var RightRingProximal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightHandRing1];
        CalculateRotation(handPoints[(int)HumanBodyBones.RightRingProximal], RightRingProximal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // RightRingIntermediate
        var RightRingIntermediate = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightHandRing2];
        CalculateRotation(handPoints[(int)HumanBodyBones.RightRingIntermediate], RightRingIntermediate, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // RightRingDistal
        var RightRingDistal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightHandRing3];
        CalculateRotation(handPoints[(int)HumanBodyBones.RightRingDistal], RightRingDistal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);















        // RightLittleProximal
        var RightLittleProximal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightHandPinky1];
        CalculateRotation(handPoints[(int)HumanBodyBones.RightLittleProximal], RightLittleProximal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // RightLittleIntermediate
        var RightLittleIntermediate = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightHandPinky2];
        CalculateRotation(handPoints[(int)HumanBodyBones.RightLittleIntermediate], RightLittleIntermediate, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // RightLittleDistal
        var RightLittleDistal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightHandPinky3];
        CalculateRotation(handPoints[(int)HumanBodyBones.RightLittleDistal], RightLittleDistal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);



        #endregion




        #region Left Hand



        // LeftThumbProximal
        var LeftThumbProximal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftHandThumb1];
        CalculateRotation(handPoints[(int)HumanBodyBones.LeftThumbProximal], LeftThumbProximal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // LeftThumbProximal
        var LeftThumbIntermediate = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftHandThumb2];
        CalculateRotation(handPoints[(int)HumanBodyBones.LeftThumbIntermediate], LeftThumbIntermediate, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // LeftThumbProximal
        var LeftThumbDistal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftHandThumb3];
        CalculateRotation(handPoints[(int)HumanBodyBones.LeftThumbDistal], LeftThumbDistal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);








        // LeftIndexProximal
        var LeftIndexProximal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftHandIndex1];
        CalculateRotation(handPoints[(int)HumanBodyBones.LeftIndexProximal], LeftIndexProximal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // LeftIndexIntermediate
        var LeftIndexIntermediate = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftHandIndex2];
        CalculateRotation(handPoints[(int)HumanBodyBones.LeftIndexIntermediate], LeftIndexIntermediate, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // LeftIndexDistal
        var LeftIndexDistal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftHandIndex3];
        CalculateRotation(handPoints[(int)HumanBodyBones.LeftIndexDistal], LeftIndexDistal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);













        // LeftMiddleProximal
        var LeftMiddleProximal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftHandMiddle1];
        CalculateRotation(handPoints[(int)HumanBodyBones.LeftMiddleProximal], LeftMiddleProximal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // LeftMiddleIntermediate
        var LeftMiddleIntermediate = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftHandMiddle2];
        CalculateRotation(handPoints[(int)HumanBodyBones.LeftMiddleIntermediate], LeftMiddleIntermediate, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // LeftMiddleDistal
        var LeftMiddleDistal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftHandMiddle3];
        CalculateRotation(handPoints[(int)HumanBodyBones.LeftMiddleDistal], LeftMiddleDistal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);













        // LeftRingProximal
        var LeftRingProximal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftHandRing1];
        CalculateRotation(handPoints[(int)HumanBodyBones.LeftRingProximal], LeftRingProximal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // LeftRingIntermediate
        var LeftRingIntermediate = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftHandRing2];
        CalculateRotation(handPoints[(int)HumanBodyBones.LeftRingIntermediate], LeftRingIntermediate, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // LeftRingDistal
        var LeftRingDistal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftHandRing3];
        CalculateRotation(handPoints[(int)HumanBodyBones.LeftRingDistal], LeftRingDistal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);















        // LeftLittleProximal
        var LeftLittleProximal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftHandPinky1];
        CalculateRotation(handPoints[(int)HumanBodyBones.LeftLittleProximal], LeftLittleProximal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // LeftLittleIntermediate
        var LeftLittleIntermediate = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftHandPinky2];
        CalculateRotation(handPoints[(int)HumanBodyBones.LeftLittleIntermediate], LeftLittleIntermediate, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);


        // LeftLittleDistal
        var LeftLittleDistal = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftHandPinky3];
        CalculateRotation(handPoints[(int)HumanBodyBones.LeftLittleDistal], LeftLittleDistal, applyDifference, new Vector3(0, 0, 0), isQuaternation, isNew, isGlobal);



        #endregion



    }

    private Quaternion CalculateRotation(CliffBodyPart cliffBodyPart)
    {
        var x = cliffBodyPart.x;
        var y = cliffBodyPart.y;
        var z = cliffBodyPart.z;
        var w = cliffBodyPart.w;

        return new Quaternion(x, y, z, w);
    }

    public override void Predict3DPose(PoseJsonVector poseJsonVector)
    {

    }

    private void CalculateRotation(JointPoint jointPoint, CliffBodyPart cliffBodyPart, bool applyDifference, Vector3 additionalRotation, bool IsQuaternation, bool isNew, bool isGlobal)
    {
        float x, y, z, w;

        if (isNew)
        {
            w = cliffBodyPart.x;
            x = cliffBodyPart.y;
            y = cliffBodyPart.z;
            z = cliffBodyPart.w;
        }
        else
        {
            x = cliffBodyPart.x;
            y = cliffBodyPart.y;
            z = cliffBodyPart.z;
            w = cliffBodyPart.w;
        }

        Debug.Log($"Angleeee x:{x},y:{y},z:{z}");

        var defaultRotation = jointPoint.DefaultRotation;
        var InitialRotation = jointPoint.InitRotation;
        var rigTransform = jointPoint.Transform;

        Vector3 eulerAngles = additionalRotation;

        var quaternion = new Quaternion(x, y, z, w);

        if (IsQuaternation)
        {
            if (isGlobal)
            {
                if (applyDifference)
                {
                    quaternion *= InitialRotation;
                    rigTransform.rotation = quaternion;
                }
                else
                {
                    rigTransform.rotation = quaternion;
                }
            }
            else
            {
                rigTransform.localRotation = quaternion;
            }

            return;
        }

        if (applyDifference)
        {
            eulerAngles += new Vector3(x * Mathf.Rad2Deg - defaultRotation.x, y * Mathf.Rad2Deg - defaultRotation.y, z * Mathf.Rad2Deg - defaultRotation.z);
        }
        else
        {
            eulerAngles += new Vector3(x * Mathf.Rad2Deg, y * Mathf.Rad2Deg, z * Mathf.Rad2Deg);
        }

        if (isGlobal)
        {
            rigTransform.eulerAngles = eulerAngles;
        }
        else
        {
            rigTransform.localEulerAngles = eulerAngles;
        }

    }


    #endregion




}
