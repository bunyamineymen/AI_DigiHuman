using System;

using UnityEngine;


[Serializable]
public enum BodyPointss : int
{
    Nose,
    LeftEyeInner,
    LeftEye,
    LeftEyeOuter,
    RightEyeInner,
    RightEye,
    RightEyeOuter,
    LeftEar,
    RightEar,
    //LeftMouth,
    //RightMouth,
    LeftShoulder,
    LeftUpperArm,
    LeftLowerArm,
    RightShoulder,
    RightUpperArm,
    RightLowerArm,
    LeftWrist,
    RightWrist,
    LeftPinky,
    RightPinky,
    LeftIndex,
    RightIndex,
    LeftThump,
    RightThump,
    LeftHip,
    RightHip,
    LeftKnee,
    RightKnee,
    LeftAnkle,
    RightAnkle,
    LeftHeel,
    RightHeel,
    LeftFootIndex,
    RightFootIndex,
    Hips,
    Spine,
    Neck,
    Head
}

[Serializable]
public struct CharacterBody
{
    public GameObject nose;
    public GameObject leftEyeInner;
    public GameObject leftEye;
    public GameObject leftEyeOuter;
    public GameObject rightEyeInner;
    public GameObject rightEye;
    public GameObject rightEyeOuter;
    public GameObject leftEar;
    public GameObject rightEar;
    public GameObject leftMouth;
    public GameObject rightMouth;

    public GameObject hips;
    public GameObject leftShoulder;
    public GameObject rightShoulder;
    public GameObject leftElbow;
    public GameObject rightElbow;
    public GameObject leftWrist;
    public GameObject rightWrist;
    public GameObject leftHip;
    public GameObject rightHip;
    public GameObject leftKnee;
    public GameObject rightKnee;
    public GameObject leftAnkle;
    public GameObject rightAnkle;
    public GameObject leftHeel;
    public GameObject rightHeel;
}



public class Pose3DMapper : CharacterMapper
{


    [SerializeField] private FrameReader frameReader;



    [Tooltip("optional")] [SerializeField] private CharacterBody characterBodyIK;
    private Transform hips;
    [SerializeField] private bool IKEnable;
    [SerializeField] private bool normalMode;
    [SerializeField] private Transform characterPlacement;
    private Vector3 headUpVector;
    private Vector3 distanceOffset;
    private JointPoint[] jointPoints;
    private GameObject[] jointsDebug;


    protected override void InitializationHumanoidPose()
    {
        //character.transform.rotation = Quaternion.identity;
        jointPoints = new JointPoint[56];

        for (var i = 0; i < jointPoints.Length; i++)
        {
            jointPoints[i] = new JointPoint();
            jointPoints[i].LastPoses = new Vector3[lowPassFilterChannels];

        }

        if (debugMode)
        {
            jointsDebug = new GameObject[33];
            for (int i = 0; i < jointsDebug.Length; i++)
            {
                jointsDebug[i] = Instantiate(debugGameObject);
            }
        }

        //////

        jointPoints[(int)HumanBodyBones.Neck].Transform = anim.GetBoneTransform(HumanBodyBones.Neck);
        jointPoints[(int)HumanBodyBones.Neck].DefaultRotation = jointPoints[(int)HumanBodyBones.Neck].Transform.localEulerAngles;
        jointPoints[(int)HumanBodyBones.Neck].InitRotation = jointPoints[(int)HumanBodyBones.Neck].Transform.rotation;

        // Right Arm
        jointPoints[(int)HumanBodyBones.RightShoulder].Transform = anim.GetBoneTransform(HumanBodyBones.RightShoulder);
        jointPoints[(int)HumanBodyBones.RightShoulder].DefaultRotation = jointPoints[(int)HumanBodyBones.RightShoulder].Transform.localEulerAngles;
        jointPoints[(int)HumanBodyBones.RightShoulder].InitRotation = jointPoints[(int)HumanBodyBones.RightShoulder].Transform.rotation;

        jointPoints[(int)HumanBodyBones.RightUpperArm].Transform = anim.GetBoneTransform(HumanBodyBones.RightUpperArm);
        jointPoints[(int)HumanBodyBones.RightUpperArm].DefaultRotation = jointPoints[(int)HumanBodyBones.RightUpperArm].Transform.localEulerAngles;
        jointPoints[(int)HumanBodyBones.RightUpperArm].InitRotation = jointPoints[(int)HumanBodyBones.RightUpperArm].Transform.rotation;

        jointPoints[(int)HumanBodyBones.RightLowerArm].Transform = anim.GetBoneTransform(HumanBodyBones.RightLowerArm);
        jointPoints[(int)HumanBodyBones.RightLowerArm].DefaultRotation = jointPoints[(int)HumanBodyBones.RightLowerArm].Transform.localEulerAngles;
        jointPoints[(int)HumanBodyBones.RightLowerArm].InitRotation = jointPoints[(int)HumanBodyBones.RightLowerArm].Transform.rotation;

        // Left Arm
        jointPoints[(int)HumanBodyBones.LeftShoulder].Transform = anim.GetBoneTransform(HumanBodyBones.LeftShoulder);
        jointPoints[(int)HumanBodyBones.LeftShoulder].DefaultRotation = jointPoints[(int)HumanBodyBones.LeftShoulder].Transform.localEulerAngles;
        jointPoints[(int)HumanBodyBones.LeftShoulder].InitRotation = jointPoints[(int)HumanBodyBones.LeftShoulder].Transform.rotation;

        jointPoints[(int)HumanBodyBones.LeftUpperArm].Transform = anim.GetBoneTransform(HumanBodyBones.LeftUpperArm);
        jointPoints[(int)HumanBodyBones.LeftUpperArm].DefaultRotation = jointPoints[(int)HumanBodyBones.LeftUpperArm].Transform.localEulerAngles;
        jointPoints[(int)HumanBodyBones.LeftUpperArm].InitRotation = jointPoints[(int)HumanBodyBones.LeftUpperArm].Transform.rotation;

        jointPoints[(int)HumanBodyBones.LeftLowerArm].Transform = anim.GetBoneTransform(HumanBodyBones.LeftLowerArm);
        jointPoints[(int)HumanBodyBones.LeftLowerArm].DefaultRotation = jointPoints[(int)HumanBodyBones.LeftLowerArm].Transform.localEulerAngles;
        jointPoints[(int)HumanBodyBones.LeftLowerArm].InitRotation = jointPoints[(int)HumanBodyBones.LeftLowerArm].Transform.rotation;

        //////

        jointPoints[(int)HumanBodyBones.RightHand].Transform = anim.GetBoneTransform(HumanBodyBones.RightHand);


        jointPoints[(int)HumanBodyBones.LeftHand].Transform = anim.GetBoneTransform(HumanBodyBones.LeftHand);

        ////Right Leg
        //jointPoints[(int)HumanBodyBones.RightHip].Transform = anim.GetBoneTransform(HumanBodyBones.RightUpperLeg);
        //jointPoints[(int)HumanBodyBones.RightKnee].Transform = anim.GetBoneTransform(HumanBodyBones.RightLowerLeg);
        //jointPoints[(int)HumanBodyBones.RightAnkle].Transform = anim.GetBoneTransform(HumanBodyBones.RightFoot);
        //jointPoints[(int)HumanBodyBones.RightFootIndex].Transform = anim.GetBoneTransform(HumanBodyBones.RightToes);

        //// Left Leg
        //jointPoints[(int)HumanBodyBones.LeftHip].Transform = anim.GetBoneTransform(HumanBodyBones.LeftUpperLeg);
        //jointPoints[(int)HumanBodyBones.LeftKnee].Transform = anim.GetBoneTransform(HumanBodyBones.LeftLowerLeg);
        //jointPoints[(int)HumanBodyBones.LeftAnkle].Transform = anim.GetBoneTransform(HumanBodyBones.LeftFoot);
        //jointPoints[(int)HumanBodyBones.LeftFootIndex].Transform = anim.GetBoneTransform(HumanBodyBones.LeftToes);

        // etc
        //jointPoints[PositionIndex.abdomenUpper.Int()].Transform = anim.GetBoneTransform(HumanBodyBones.Spine);
        // jointPoints[(int) HumanBodyBones.Hips].Transform = hips.transform;



        jointPoints[(int)HumanBodyBones.Hips].Transform = anim.GetBoneTransform(HumanBodyBones.Hips);



        jointPoints[(int)HumanBodyBones.Head].Transform = anim.GetBoneTransform(HumanBodyBones.Head);
        jointPoints[(int)HumanBodyBones.Head].DefaultRotation = jointPoints[(int)HumanBodyBones.Head].Transform.localEulerAngles;







        jointPoints[(int)HumanBodyBones.Spine].Transform = anim.GetBoneTransform(HumanBodyBones.Spine);
        jointPoints[(int)HumanBodyBones.Spine].DefaultRotation = jointPoints[(int)HumanBodyBones.Spine].Transform.localEulerAngles;



        // Child Settings
        // Right Arm
        jointPoints[(int)HumanBodyBones.RightShoulder].Child = jointPoints[(int)HumanBodyBones.RightUpperArm];
        jointPoints[(int)HumanBodyBones.RightUpperArm].Child = jointPoints[(int)HumanBodyBones.RightLowerArm];
        jointPoints[(int)HumanBodyBones.RightUpperArm].Parent = jointPoints[(int)HumanBodyBones.RightShoulder];

        // Left Arm
        jointPoints[(int)HumanBodyBones.LeftShoulder].Child = jointPoints[(int)HumanBodyBones.LeftUpperArm];
        jointPoints[(int)HumanBodyBones.LeftLowerArm].Child = jointPoints[(int)HumanBodyBones.LeftHand];
        jointPoints[(int)HumanBodyBones.LeftUpperArm].Parent = jointPoints[(int)HumanBodyBones.LeftShoulder];

        return;

        // Fase



        // Right Leg
        //jointPoints[(int)HumanBodyBones.RightHip].Child = jointPoints[(int)HumanBodyBones.RightKnee];
        //jointPoints[(int)HumanBodyBones.RightKnee].Child = jointPoints[(int)HumanBodyBones.RightAnkle];
        //jointPoints[(int)HumanBodyBones.RightAnkle].Child = jointPoints[(int)HumanBodyBones.RightFootIndex];
        //// jointPoints[(int) HumanBodyBones.RightKnee].Parent = jointPoints[(int) HumanBodyBones.RightHip];
        //jointPoints[(int)HumanBodyBones.RightAnkle].Parent = jointPoints[(int)HumanBodyBones.RightKnee];

        //// Left Leg
        //jointPoints[(int)HumanBodyBones.LeftHip].Child = jointPoints[(int)HumanBodyBones.LeftKnee];
        //jointPoints[(int)HumanBodyBones.LeftKnee].Child = jointPoints[(int)HumanBodyBones.LeftAnkle];
        //jointPoints[(int)HumanBodyBones.LeftAnkle].Child = jointPoints[(int)HumanBodyBones.LeftFootIndex];
        //// jointPoints[(int) HumanBodyBones.LeftKnee].Parent = jointPoints[(int) HumanBodyBones.LeftHip];
        //jointPoints[(int)HumanBodyBones.LeftAnkle].Parent = jointPoints[(int)HumanBodyBones.LeftKnee];


        // etc
        //jointPoints[(int)HumanBodyBones.Neck].Child = jointPoints[(int)HumanBodyBones.Neck];
        //jointPoints[(int)HumanBodyBones.Neck].Child = jointPoints[(int)HumanBodyBones.Head];
        //jointPoints[(int)HumanBodyBones.Head].Child = jointPoints[(int)HumanBodyBones.Nose];


        //for (int i = 0; i < jointPoints.Length; i++)
        //{
        //    if (jointPoints[i].Child != null)
        //    {
        //        if (jointPoints[i].Child.Transform != null)
        //        {
        //            jointPoints[i].DistanceFromChild = Vector3.Distance(jointPoints[i].Child.Transform.position,
        //                jointPoints[i].Transform.position);
        //        }
        //    }
        //}



        //// Set Inverse
        //Vector3 a = jointPoints[(int)HumanBodyBones.LeftHip].Transform.position;
        //Vector3 b = jointPoints[(int)HumanBodyBones.Spine].Transform.position;
        //hips = jointPoints[(int)HumanBodyBones.Hips].Transform;
        //Vector3 c = jointPoints[(int)HumanBodyBones.RightHip].Transform.position;
        //var forward = b.TriangleNormal(a, c);



        //foreach (var jointPoint in jointPoints)
        //{
        //    if (jointPoint.Transform != null)
        //    {
        //        jointPoint.InitRotation = jointPoint.Transform.rotation;
        //    }

        //    if (jointPoint.Child != null)
        //    {
        //        jointPoint.Inverse = Quaternion.Inverse(Quaternion.LookRotation(jointPoint.Transform.position - jointPoint.Child.Transform.position, forward));
        //        jointPoint.InverseRotation = jointPoint.Inverse * jointPoint.InitRotation;
        //    }
        //}



        ////Hip and Spine
        ////var hip = jointPoints[(int)HumanBodyBones.Hips];
        ////var spine = jointPoints[(int)HumanBodyBones.Spine];
        ////hip.Inverse = Quaternion.Inverse(Quaternion.LookRotation(forward, spine.Transform.position - hip.Transform.position));
        ////hip.InverseRotation = hip.Inverse * hip.InitRotation;

        ////if (spine.Transform != null)
        ////{
        ////    spine.Inverse = Quaternion.Inverse(Quaternion.LookRotation(
        ////        spine.Transform.position.TriangleNormal(jointPoints[(int)HumanBodyBones.RightShoulder].Transform.position,
        ////            jointPoints[(int)HumanBodyBones.LeftShoulder].Transform.position),
        ////        jointPoints[(int)HumanBodyBones.Neck].Transform.position - spine.Transform.position));
        ////    spine.InverseRotation = spine.Inverse * spine.InitRotation;
        ////}

        //// For Head Rotation
        //var head = jointPoints[(int)HumanBodyBones.Head];
        //head.InitRotation = jointPoints[(int)HumanBodyBones.Head].Transform.rotation;
        //var gaze = head.Transform.up;

        ////Debug.Log(gaze);

        //head.Inverse = Quaternion.Inverse(Quaternion.LookRotation(gaze));
        //// head.InverseRotation = head.Inverse * head.InitRotation; //TODO check why?

        //head.InverseRotation = head.InitRotation;
        //headUpVector = head.Transform.up;


        ////feet setup
        //var r_feet = jointPoints[(int)HumanBodyBones.RightAnkle];

        //r_feet.Inverse = Quaternion.Inverse(Quaternion.LookRotation(r_feet.Transform.position - jointPoints[(int)HumanBodyBones.RightFootIndex].Transform.position, jointPoints[(int)HumanBodyBones.RightKnee].Transform.position - r_feet.Transform.position));
        //r_feet.InverseRotation = r_feet.Inverse * r_feet.InitRotation;

        //var l_feet = jointPoints[(int)HumanBodyBones.LeftAnkle];
        //l_feet.Inverse = Quaternion.Inverse(Quaternion.LookRotation(l_feet.Transform.position - jointPoints[(int)HumanBodyBones.LeftFootIndex].Transform.position, jointPoints[(int)HumanBodyBones.LeftKnee].Transform.position - l_feet.Transform.position));
        //l_feet.InverseRotation = l_feet.Inverse * l_feet.InitRotation;

        ////
        //// var lHand = jointPoints[PositionIndex.lHand.Int()];
        //// var lf = TriangleNormal(lHand.Pos3D, jointPoints[PositionIndex.lMid1.Int()].Pos3D, jointPoints[PositionIndex.lThumb2.Int()].Pos3D);
        //// lHand.InitRotation = lHand.Transform.rotation;
        //// lHand.Inverse = Quaternion.Inverse(Quaternion.LookRotation(jointPoints[PositionIndex.lThumb2.Int()].Transform.position - jointPoints[PositionIndex.lMid1.Int()].Transform.position, lf));
        //// lHand.InverseRotation = lHand.Inverse * lHand.InitRotation;
        ////
        //// var rHand = jointPoints[PositionIndex.rHand.Int()];
        //// var rf = TriangleNormal(rHand.Pos3D, jointPoints[PositionIndex.rThumb2.Int()].Pos3D, jointPoints[PositionIndex.rMid1.Int()].Pos3D);
        //// rHand.InitRotation = jointPoints[PositionIndex.rHand.Int()].Transform.rotation;
        //// rHand.Inverse = Quaternion.Inverse(Quaternion.LookRotation(jointPoints[PositionIndex.rThumb2.Int()].Transform.position - jointPoints[PositionIndex.rMid1.Int()].Transform.position, rf));
        //// rHand.InverseRotation = rHand.Inverse * rHand.InitRotation;

        //// distanceOffset = character.transform.position

        //for (int i = 0; i < jointPoints.Length; i++)
        //{
        //    if (jointPoints[i].Transform != null)
        //        jointPoints[i].LandmarkPose = jointPoints[i].Transform.position;
        //}

        //Debug.Log("wtf");
        ////character.transform.rotation = characterPlacement.rotation;
        //hips.position = characterPlacement.position;

    }



    private void UpdateNormalMode(BodyPartVector[] bodyPartVectors)
    {
        ////Debug.Log($"bodyPartVectors {bodyPartVectors.Length}");

        //for (int i = 0; i < bodyPartVectors.Length; i++)
        //{
        //    if (i == 23 || i == 24)
        //    {
        //        continue;
        //    }

        //    if (bodyPartVectors[i].visibility > 0.2f)
        //    {
        //        jointPoints[i].LandmarkPose = bodyPartVectors[i].position;
        //    }
        //}

        ////transfer position metrics
        //for (int i = 0; i < jointPoints.Length && i < bodyPartVectors.Length; i++)
        //{
        //    JointPoint bone = jointPoints[i];

        //    if (bone.Transform != null)
        //    {
        //        bone.WorldPos = bone.Transform.position;
        //    }
        //}



        //for (int i = 0; i < jointPoints.Length && i < bodyPartVectors.Length; i++)
        //{
        //    JointPoint bone = jointPoints[i];

        //    if (bone.Child != null)
        //    {
        //        if (bone.Child.Transform != null)
        //        {
        //            JointPoint child = bone.Child;
        //            float distance = bone.DistanceFromChild;

        //            Vector3 direction = (-bone.LandmarkPose + child.LandmarkPose) / (-bone.LandmarkPose + child.LandmarkPose).magnitude;
        //            child.WorldPos = bone.Transform.position + direction * distance;
        //        }
        //    }
        //}



        //if (enableKalmanFilter)
        //{
        //    for (int i = 0; i < jointPoints.Length; i++)
        //    {
        //        if (jointPoints[i].Transform != null)
        //            KalmanUpdate(jointPoints[i]);
        //    }
        //}
        //else
        //{
        //    for (int i = 0; i < jointPoints.Length; i++)
        //    {
        //        jointPoints[i].FilteredPos = jointPoints[i].WorldPos;
        //    }
        //}

        //if (useLowPassFilter)
        //{
        //    foreach (var jp in jointPoints)
        //    {
        //        jp.LastPoses[0] = jp.FilteredPos;
        //        for (var i = 1; i < jp.LastPoses.Length; i++)
        //        {
        //            jp.LastPoses[i] = jp.LastPoses[i] * lowPassParam + jp.LastPoses[i - 1] * (1f - lowPassParam);
        //        }
        //        jp.FilteredPos = jp.LastPoses[jp.LastPoses.Length - 1];
        //    }
        //}

        ////Rotation


        ////setting hip & spine rotation
        //Vector3 rightHipPosition = bodyPartVectors[(int)HumanBodyBones.RightHip].position;
        //Vector3 spine = bodyPartVectors[(int)HumanBodyBones.Spine].position;
        //Vector3 hip = bodyPartVectors[(int)HumanBodyBones.Hips].position;
        //Vector3 leftHipPosition = bodyPartVectors[(int)HumanBodyBones.LeftHip].position;
        //Vector3 rightShoulder = bodyPartVectors[(int)HumanBodyBones.RightShoulder].position;
        //Vector3 leftShoulder = bodyPartVectors[(int)HumanBodyBones.LeftShoulder].position;
        //Vector3 hipsUpward = spine - hip;
        //Vector3 spineUpward = bodyPartVectors[(int)HumanBodyBones.Neck].position - spine;



        //var rotationValue = Quaternion.LookRotation(spine.TriangleNormal(rightShoulder, leftShoulder), spineUpward);



        //// HIP
        //var hipsRot1 = Quaternion.LookRotation(spine.TriangleNormal(leftHipPosition, rightHipPosition), hipsUpward);
        //var hipsRot2 = jointPoints[(int)HumanBodyBones.Hips].InverseRotation;

        //var hipRotation = hipsRot1 * hipsRot2;

        //jointPoints[(int)HumanBodyBones.Hips].Transform.rotation = hipRotation;

        ////jointPoints[(int)HumanBodyBones.Hips].Transform.rotation = hipsRot1;



        //// SPINE

        //var spineRot1 = Quaternion.LookRotation(spine.TriangleNormal(rightShoulder, leftShoulder), spineUpward);
        //var spineRot2 = jointPoints[(int)HumanBodyBones.Spine].InverseRotation;

        //jointPoints[(int)HumanBodyBones.Spine].Transform.rotation = spineRot1 * spineRot2;


        //#region Head

        //// HEAD
        ////Vector3 mouth = (bodyPartVectors[(int)HumanBodyBones.LeftMouth].position +
        ////                 bodyPartVectors[(int)HumanBodyBones.RightMouth].position) / 2.0f;

        //Vector3 mouth = Vector3.zero;

        //Vector3 lEye = bodyPartVectors[(int)HumanBodyBones.LeftEye].position;
        //Vector3 rEye = bodyPartVectors[(int)HumanBodyBones.RightEye].position;

        //Vector3 eyeAverage = (lEye + rEye) / 2;

        //var gaze = lEye.TriangleNormal(mouth, rEye);

        //Vector3 nose = bodyPartVectors[(int)HumanBodyBones.Nose].position;
        //Vector3 rEar = bodyPartVectors[(int)HumanBodyBones.RightEar].position;
        //Vector3 lEar = bodyPartVectors[(int)HumanBodyBones.LeftEar].position;
        //var head = jointPoints[(int)HumanBodyBones.Head];
        //Vector3 normal = nose.TriangleNormal(rEar, lEar);
        //head.Transform.rotation = Quaternion.LookRotation(gaze, normal) * head.InverseRotation;

        //Plane plane = new Plane(lEye, rEye, mouth);
        //Vector3 faceUpDir = eyeAverage - mouth;

        //#endregion



        //// rotate each of bones
        //Vector3 forward = jointPoints[(int)HumanBodyBones.Hips].Transform.forward;

        //Vector3 leftHipFilteredPos = jointPoints[(int)HumanBodyBones.LeftHip].FilteredPos;
        //Vector3 rightHipFilteredPos = jointPoints[(int)HumanBodyBones.RightHip].FilteredPos;
        //forward = jointPoints[(int)HumanBodyBones.Spine].FilteredPos.TriangleNormal(leftHipFilteredPos, rightHipFilteredPos);


        //foreach (var jointPoint in jointPoints)
        //{

        //    if (jointPoint == null)
        //        continue;

        //    if (jointPoint.Parent != null)
        //    {
        //        Vector3 fv = jointPoint.Parent.FilteredPos - jointPoint.FilteredPos;
        //        jointPoint.Transform.rotation =
        //            Quaternion.LookRotation(jointPoint.FilteredPos - jointPoint.Child.FilteredPos, fv)
        //            * jointPoint.InverseRotation;
        //    }
        //    else if (jointPoint.Child != null)
        //    {
        //        jointPoint.Transform.rotation =
        //            Quaternion.LookRotation((jointPoint.FilteredPos - jointPoint.Child.FilteredPos).normalized, forward)
        //            * jointPoint.InverseRotation;
        //    }

        //    //continue;





        //    //if (jointPoint.Parent != null)
        //    //{
        //    //    Vector3 fv = jointPoint.Parent.Transform.position - jointPoint.Transform.position;
        //    //    jointPoint.Transform.rotation = Quaternion.LookRotation(jointPoint.Transform.position - jointPoint.Child.Transform.position, fv) * jointPoint.InverseRotation;
        //    //}
        //    //else if (jointPoint.Child != null)
        //    //{
        //    //    jointPoint.Transform.rotation = Quaternion.LookRotation(jointPoint.Transform.position - jointPoint.Child.Transform.position, forward) * jointPoint.InverseRotation;
        //    //}
        //    //continue;
        //    //if (jointPoint.Parent != null)
        //    //{
        //    //    var fv = jointPoint.Parent.Transform.position - jointPoint.Transform.position;
        //    //    jointPoint.Transform.rotation = Quaternion.LookRotation(jointPoint.Transform.position - jointPoint.Child.Transform.position, fv);
        //    //}
        //    //else if (jointPoint.Child != null)
        //    //{
        //    //    jointPoint.Transform.rotation = Quaternion.LookRotation(jointPoint.Transform.position - jointPoint.Child.Transform.position, forward);
        //    //}

        //}


        ////Calculate feet rotation
        //Vector3 r_ankle = bodyPartVectors[(int)HumanBodyBones.RightAnkle].position;
        //Vector3 r_toe = bodyPartVectors[(int)HumanBodyBones.RightFootIndex].position;
        //Vector3 r_knee = bodyPartVectors[(int)HumanBodyBones.RightKnee].position;

        //JointPoint r_ankleT = jointPoints[(int)HumanBodyBones.RightAnkle];


        ////r_ankleT.Transform.rotation =
        ////    Quaternion.LookRotation(r_ankle - r_toe, r_knee - r_ankle)
        ////    * r_ankleT.InverseRotation;




        //Vector3 l_ankle = bodyPartVectors[(int)HumanBodyBones.LeftAnkle].position;
        //Vector3 l_toe = bodyPartVectors[(int)HumanBodyBones.LeftFootIndex].position;
        //Vector3 l_knee = bodyPartVectors[(int)HumanBodyBones.LeftKnee].position;

        //JointPoint l_ankleT = jointPoints[(int)HumanBodyBones.LeftAnkle];

        ////l_ankleT.Transform.rotation =
        ////    Quaternion.LookRotation(l_ankle - l_toe, l_knee - l_ankle)
        ////    * l_ankleT.InverseRotation;




        //// for (int i = 0; i < jointPoints.Length && i < bodyPartVectors.Length; i++)
        //// {
        ////     JointPoint bone = jointPoints[i];
        ////
        ////     if (bone.Child != null)
        ////     {
        ////         if (bone.Child.Transform != null)
        ////         {
        ////             JointPoint child = bone.Child;
        ////             child.Transform.position = child.WorldPos;
        ////         }
        ////     }
        //// }
        //// Vector3 a1 = bodyPartVectors[(int) HumanBodyBones.Nose].position;
        //// Vector3 b1 = bodyPartVectors[(int) HumanBodyBones.RightEar].position;
        //// Vector3 c1 = bodyPartVectors[(int) HumanBodyBones.LeftHip].position;
        //// float yDegree = Vector3.Angle(b1.TriangleNormal(a1, c1), Vector3.up);
        //// var head = jointPoints[(int) HumanBodyBones.Head];
        //// Vector3 headAngle = head.Transform.eulerAngles;
        //// headAngle.y = yDegree;
        //// head.Transform.eulerAngles = headAngle;

        //// Vector3 a1 = bodyPartVectors[(int) HumanBodyBones.RightEye].position;
        //// Vector3 b1 = bodyPartVectors[(int) HumanBodyBones.LeftEye].position;
        //// Vector3 c1 = (bodyPartVectors[(int) HumanBodyBones.LeftMouth].position + bodyPartVectors[(int) HumanBodyBones.RightMouth].position);
        //// float yDegree = Vector3.Angle(c1.TriangleNormal(a1, b1), Vector3.forward);
        //// var head = jointPoints[(int) HumanBodyBones.Head];
        //// Vector3 headAngle = head.Transform.eulerAngles;
        //// headAngle.y = -yDegree;
        //// head.Transform.eulerAngles = headAngle;
        ////jointPoints[(int) HumanBodyBones.Hips].Transform.position = characterPlacement;
    }

    ////placing and rotating bones with the help of IK algorithm
    //private void UpdateModeIK(BodyPartVector[] bodyPartVectors)
    //{

    //    //setting hips position
    //    characterBodyIK.hips.transform.position = bodyPartVectors[(int)HumanBodyBones.Hips].position;

    //    //setting hip rotation
    //    Vector3 a = bodyPartVectors[(int)HumanBodyBones.LeftShoulder].position;
    //    Vector3 b = jointPoints[(int)HumanBodyBones.Hips].Transform.position;
    //    Vector3 c = bodyPartVectors[(int)HumanBodyBones.RightShoulder].position;
    //    jointPoints[(int)HumanBodyBones.Hips].Transform.rotation = Quaternion.LookRotation(a.TriangleNormal(b, c)) *
    //                                                            jointPoints[(int)HumanBodyBones.Hips].InverseRotation;

    //    //IK
    //    //characterBodyIK.leftElbow.transform.position = bodyPartVectors[(int)HumanBodyBones.LeftElbow].position;
    //    //characterBodyIK.rightElbow.transform.position = bodyPartVectors[(int)HumanBodyBones.RightElbow].position;
    //    characterBodyIK.leftAnkle.transform.position = bodyPartVectors[(int)HumanBodyBones.LeftAnkle].position;
    //    characterBodyIK.rightAnkle.transform.position = bodyPartVectors[(int)HumanBodyBones.RightAnkle].position;
    //    characterBodyIK.leftWrist.transform.position = bodyPartVectors[(int)HumanBodyBones.LeftWrist].position;
    //    characterBodyIK.rightWrist.transform.position = bodyPartVectors[(int)HumanBodyBones.RightWrist].position;
    //    //return;

    //    characterBodyIK.leftShoulder.transform.position = bodyPartVectors[(int)HumanBodyBones.LeftShoulder].position;
    //    characterBodyIK.rightShoulder.transform.position = bodyPartVectors[(int)HumanBodyBones.RightShoulder].position;

    //    characterBodyIK.leftHip.transform.position = bodyPartVectors[(int)HumanBodyBones.LeftHip].position;
    //    characterBodyIK.rightHip.transform.position = bodyPartVectors[(int)HumanBodyBones.RightHip].position;
    //    characterBodyIK.leftKnee.transform.position = bodyPartVectors[(int)HumanBodyBones.LeftKnee].position;
    //    characterBodyIK.rightKnee.transform.position = bodyPartVectors[(int)HumanBodyBones.RightKnee].position;
    //    characterBodyIK.leftHeel.transform.position = bodyPartVectors[(int)HumanBodyBones.LeftHeel].position;
    //    characterBodyIK.rightHeel.transform.position = bodyPartVectors[(int)HumanBodyBones.RightHeel].position;

    //    PoseJson poseJson = frameReader.currentPoseJson;
    //    BodyPart[] bodyParts = poseJson.predictions;
    //    characterBodyIK.leftShoulder.transform.position = new Vector3(-bodyParts[11].x,
    //        -bodyParts[11].y, bodyParts[11].z);

    //    characterBodyIK.rightShoulder.transform.position = new Vector3(-bodyParts[12].x,
    //        -bodyParts[12].y, bodyParts[12].z);

    //    characterBodyIK.leftElbow.transform.position = new Vector3(-bodyParts[13].x,
    //        -bodyParts[13].y, bodyParts[13].z);

    //    characterBodyIK.rightElbow.transform.position = new Vector3(-bodyParts[14].x,
    //        -bodyParts[14].y, bodyParts[14].z);

    //    characterBodyIK.leftWrist.transform.position = new Vector3(-bodyParts[15].x,
    //        -bodyParts[15].y, bodyParts[15].z);

    //    characterBodyIK.rightWrist.transform.position = new Vector3(-bodyParts[16].x,
    //        -bodyParts[16].y, bodyParts[16].z);

    //    characterBodyIK.leftHip.transform.position = new Vector3(-bodyParts[23].x,
    //        -bodyParts[23].y, bodyParts[23].z);

    //    characterBodyIK.rightHip.transform.position = new Vector3(-bodyParts[24].x,
    //        -bodyParts[24].y, bodyParts[24].z);

    //    characterBodyIK.hips.transform.position = (characterBodyIK.rightHip.transform.position +
    //                                             characterBodyIK.leftHip.transform.position +
    //                                             characterBodyIK.leftShoulder.transform.position +
    //                                             characterBodyIK.rightShoulder.transform.position) / 4;


    //    characterBodyIK.leftKnee.transform.position = new Vector3(-bodyParts[25].x,
    //        -bodyParts[25].y, bodyParts[25].z);

    //    characterBodyIK.rightKnee.transform.position = new Vector3(-bodyParts[26].x,
    //        -bodyParts[26].y, bodyParts[26].z);

    //    characterBodyIK.leftHeel.transform.position = new Vector3(-bodyParts[27].x,
    //        -bodyParts[27].y, bodyParts[27].z);

    //    characterBodyIK.rightHeel.transform.position = new Vector3(-bodyParts[28].x,
    //        -bodyParts[28].y, bodyParts[28].z);

    //    characterBodyIK.leftAnkle.transform.position = new Vector3(-bodyParts[27].x,
    //        -bodyParts[27].y, bodyParts[27].z);

    //    characterBodyIK.rightAnkle.transform.position = new Vector3(-bodyParts[28].x,
    //        -bodyParts[28].y, bodyParts[28].z);

    //}

    public override void Predict3DPose(PoseJsonVector poseJsonVector)
    {
        BodyPartVector[] bodyPartVectors = poseJsonVector.predictions;
        if (bodyPartVectors == null)
            return;
        if (bodyPartVectors.Length == 0)
            return;

        if (debugMode)
        {
            for (int i = 0; i < bodyPartVectors.Length; i++)
            {
                jointsDebug[i].transform.position = bodyPartVectors[i].position;
            }
        }
        try
        {
            character.transform.rotation = Quaternion.identity;

            if (normalMode)
            {
                UpdateNormalMode(bodyPartVectors);
            }
            else if (IKEnable)
            {
                //UpdateModeIK(bodyPartVectors);
            }

            //character.transform.rotation = characterPlacement.rotation;
            hips.position = characterPlacement.position;

        }
        catch (Exception e)
        {
            //Debug.LogError("Pose problem");
            //throw e;
        }
    }

    #region Cliff

    public void PredictCliff3DPose(CliffFrame cliffFrame)
    {


        // Neck
        var Neck = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.Neck];
        CalculateRotation(jointPoints[(int)HumanBodyBones.Neck], Neck, true, Vector3.zero, false, true);



        // Head
        var Head = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.Head];
        CalculateRotation(jointPoints[(int)HumanBodyBones.Head], Head, true, Vector3.zero, false, true);




        // RightShoulder
        var RightShoulder = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightShoulder];
        CalculateRotation(jointPoints[(int)HumanBodyBones.RightShoulder], RightShoulder, true, Vector3.zero, false, true);


        return;


        // RightArm
        var RightArm = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightArm];
        CalculateRotation(jointPoints[(int)HumanBodyBones.RightUpperArm], RightArm, true, Vector3.zero, false, true);

        // RightForeArm
        var RightForeArm = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightForeArm];
        CalculateRotation(jointPoints[(int)HumanBodyBones.RightLowerArm], RightForeArm, true, Vector3.zero, false, true);

        return;


        //return;

        // try the quaternation struct....

        //// LeftShoulder
        //var LeftShoulder = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftShoulder];
        //CalculateRotation(jointPoints[(int)HumanBodyBones.LeftShoulder], LeftShoulder, true, new Vector3(0, 0, 0), false, true);

        //// LeftArm
        //var LeftArm = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftArm];
        //CalculateRotation(jointPoints[(int)HumanBodyBones.LeftUpperArm], LeftArm, true, Vector3.zero, false, true);

        //// LeftForeArm
        //var LeftForeArm = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftForeArm];
        //CalculateRotation(jointPoints[(int)HumanBodyBones.LeftLowerArm], LeftForeArm, true, new Vector3(0, 0, 0), false, true);

        //return;

        #region Comment

        // Spine1
        //var Spine1 = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.Spine1];
        //CalculateRotation(jointPoints[(int)HumanBodyBones.Spine], Spine1);

        //// Hips
        //var Hips = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.Hips];
        //CalculateRotation(jointPoints[(int)HumanBodyBones.Hips], Hips);

        //// LeftUpLeg
        //var LeftUpLeg = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftUpLeg];
        //jointPoints[(int)HumanBodyBones.LeftHip].Transform.localRotation = CalculateRotation(LeftUpLeg);

        //// RightUpLeg
        //var RightUpLeg = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightUpLeg];
        //jointPoints[(int)HumanBodyBones.RightHip].Transform.localRotation = CalculateRotation(RightUpLeg);

        //// LeftLeg
        //var LeftLeg = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftLeg];
        //jointPoints[(int)HumanBodyBones.LeftKnee].Transform.localRotation = CalculateRotation(LeftLeg);

        //// RightLeg
        //var RightLeg = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightLeg];
        //jointPoints[(int)HumanBodyBones.RightKnee].Transform.localRotation = CalculateRotation(RightLeg);

        ////// Spine2
        ////var Spine2 = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.Spine2];
        ////jointPoints[(int)HumanBodyBones.].Transform.localRotation = CalculateRotation(Spine2);


        //// LeftFoot
        //var LeftFoot = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftFoot];
        //jointPoints[(int)HumanBodyBones.LeftWrist].Transform.localRotation = CalculateRotation(LeftFoot);

        //// RightFoot
        //var RightFoot = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightFoot];
        //jointPoints[(int)HumanBodyBones.RightWrist].Transform.localRotation = CalculateRotation(RightFoot);


        ////// Hips
        ////var Spine3 = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.Spine3];
        ////jointPoints[(int)HumanBodyBones.Hips].Transform.localRotation = CalculateRotation(Spine3);


        ////// Hips
        ////var LeftToeBase = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.LeftToeBase];
        ////jointPoints[(int)HumanBodyBones.Hips].Transform.localRotation = CalculateRotation(LeftToeBase);



        ////// Hips
        ////var RightToeBase = cliffFrame.BodyPartRotations[(int)BodyPartsOfCliff.RightToeBase];
        ////jointPoints[(int)HumanBodyBones.Hips].Transform.localRotation = CalculateRotation(RightToeBase);


        #endregion


    }

    private void CalculateRotation(JointPoint jointPoint, CliffBodyPart cliffBodyPart, bool applyDifference, Vector3 additionalRotation, bool IsQuaternation,bool isNew)
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

        Vector3 localEulerAngles = additionalRotation;

        if (IsQuaternation)
        {

            var quaternion = new Quaternion(x, y, z, w);

            rigTransform.localRotation = quaternion * InitialRotation;
            //rigTransform.localRotation = quaternion;

            return;
        }

        if (applyDifference)
        {
            localEulerAngles += new Vector3(x * Mathf.Rad2Deg - defaultRotation.x, y * Mathf.Rad2Deg - defaultRotation.y, z * Mathf.Rad2Deg - defaultRotation.z);
        }
        else
        {
            localEulerAngles += new Vector3(x * Mathf.Rad2Deg, y * Mathf.Rad2Deg, z * Mathf.Rad2Deg);
        }



        rigTransform.localEulerAngles = localEulerAngles;

    }

    #endregion


}






//setting position of each bone
//jointPoints[(int)HumanBodyBones.Spine].Transform.position = bodyPartVectors[(int)HumanBodyBones.Spine].position;
//jointPoints[(int)HumanBodyBones.Hips].Transform.position = bodyPartVectors[(int)HumanBodyBones.Hips].position;