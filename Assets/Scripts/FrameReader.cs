using Newtonsoft.Json;

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Video;

public class FrameReader : MonoBehaviour
{

    public CliffConfiguration CliffConfiguration;

    #region Variables

    [Header("Requirements")]
    public Pose3DMapper pose3DMapper;
    public HandsPreprocessor handPose;
    public FacialExpressionHandler facialExpressionHandler;

    public VideoPlayer videoPlayer;

    private List<FrameData> frameData;
    private FrameData currentFrameData;

    private bool framesLoaded = false;

    [Header("Fractions to multiply by pose estimates")]
    public float fraction = 1.2f;
    public float fractionX = 1.2f;
    public float fractionY = 1.2f;
    public float fractionZ = 1.2f;
    [SerializeField] private bool enableVideoAspectRatioEffector;
    private float videoFractionX = 1;
    private float videoFractionY = 1;
    private float videoFractionZ = 1;

    [Header("Frame rate")]
    [SerializeField] private float nextFrameTime;

    private int currentAnimationSlot = 0;

    //Body pose
    private List<PoseJsonVector> estimatedPoses;
    [HideInInspector] public PoseJson currentPoseJson;
    [HideInInspector] public PoseJsonVector currentPoseJsonVector;
    [HideInInspector] public PoseJsonVector currentPoseJsonVectorNew;
    [HideInInspector] public int poseIndex;


    //Hand pose
    private List<HandJsonVector> estimatedHandPose;
    [HideInInspector] public HandJsonVector currentHandJsonVector;
    [HideInInspector] public HandJsonVector currentHandJsonVectorNew;
    [HideInInspector] public int handIndex;

    //facial mocap
    private List<FaceJson> estimatedFacialMocap;
    [HideInInspector] public FaceJson currentFaceJson;
    [HideInInspector] public FaceJson currentFaceJsonNew;
    [HideInInspector] public int faceIndex;


    [Header("3D Character")]
    [SerializeField] private GameObject character;
    [SerializeField] private bool enableFace;
    [SerializeField] private bool enableHands = true;

    [Header("PlayController")]
    public bool pause = true;

    [SerializeField] private Slider slider;

    [SerializeField] private bool enableVideo;


    [Header("Camera Zoom")]
    [SerializeField] private Transform bodyZoomCameraPlace;
    [SerializeField] private Transform faceZoomCameraPlace;
    [SerializeField] private Camera camera;

    [Header("Debug")]
    [SerializeField] private bool debug;

    [SerializeField] private bool readFromFileHand;
    [SerializeField] private TextAsset jsonTestHand;

    [SerializeField] private bool readFromFace;
    [SerializeField] private TextAsset jsonTestFace;

    [SerializeField] private bool readFromFilePose;
    [SerializeField] private TextAsset jsonTestPose;

    [SerializeField] private bool enableFileSeriesReader;
    [SerializeField] private string path = "C:\\Danial\\Projects\\Danial\\DigiHuman\\Backend\\hand_json\\";
    [SerializeField] private bool onlyCurrentIndex;

    [Header("Recording")]
    [SerializeField] private GameObject recorder;

    private Quaternion characterRotation;

    private float timer = 0;
    private string jsonTest;
    [SerializeField] private int fileIndex = 1;

    #endregion


    #region Monobehaviours

    private void Start()
    {
        estimatedPoses = new List<PoseJsonVector>();
        estimatedFacialMocap = new List<FaceJson>();
        estimatedHandPose = new List<HandJsonVector>();
        frameData = new List<FrameData>();
        //characterRotation = character.transform.rotation;
        SetBodyZoomCamera();
        videoPlayer.Prepare();
        videoPlayer.Play();
        videoPlayer.frame = 0;
        videoPlayer.Pause();

        CliffStartMethod();
    }

    private void FixedUpdate()
    {

        Cliff_FixedUpdate();

        return;

        if (!pause)
        {
            timer += Time.fixedDeltaTime;
        }

        currentAnimationSlot = (int)slider.value;

        //Debug.Log($"currentAnimationSlot: {currentAnimationSlot}");

        if (debug)
        {
            //DebugMethod();

            return;
        }

        if (currentAnimationSlot >= frameData.Count)
        {
            OnAnimationPlayFinish();
            return;
        }



        #region Comment1

        //if (!pause && enableVideo)
        //{
        //    if (videoPlayer.url != "")
        //    {
        //        Debug.Log(videoPlayer.time);
        //        Debug.Log(currentFaceJson.time);

        //        if (videoPlayer.time < currentFaceJson.time / 1000.0f)
        //            return;

        //        PlayVideo();
        //    }
        //} 

        #endregion



        //Current Frame data 

        currentFrameData = frameData[currentAnimationSlot];

        //Debug.Log($"currentFrameData: {currentFrameData}");


        //Body
        currentPoseJsonVector = currentPoseJsonVectorNew;
        currentPoseJsonVectorNew = currentFrameData.poseData;

        //Hand
        currentHandJsonVector = currentHandJsonVectorNew;
        currentHandJsonVectorNew = currentFrameData.handData;

        currentFaceJson = currentFrameData.faceData;



        #region Comment2

        if (timer >= nextFrameTime)
        {
            if (debug)
            {
                videoPlayer.frame = frameData[currentAnimationSlot].frame;
                videoPlayer.Play();
                videoPlayer.Pause();
            }

            timer = 0;
            currentAnimationSlot++;

        }

        #endregion


        try
        {
            //character.transform.rotation = Quaternion.identity;


            //-------- Body Pose ------
            if (currentPoseJsonVector != null)
            {

                //TODO change maybe looking for 5 frames later!
                if (currentPoseJsonVectorNew != null)
                {
                    //for each bone position in the current frame
                    for (int i = 0; i < currentPoseJsonVector.predictions.Length; i++)
                    {
                        currentPoseJsonVector.predictions[i].position = Vector3.Lerp(
                            currentPoseJsonVector.predictions[i].position,
                            currentPoseJsonVectorNew.predictions[i].position,
                            timer / nextFrameTime);
                    }
                }

                pose3DMapper.Predict3DPose(currentPoseJsonVector);
            }


            //----- Hands -----
            if (currentHandJsonVector != null && enableHands)
            {
                if (currentHandJsonVectorNew != null)
                {
                    if (currentHandJsonVector.handsR.Length == currentHandJsonVectorNew.handsR.Length)
                        //for each bone position in the current frame
                        for (int i = 0; i < currentHandJsonVector.handsR.Length; i++)
                        {
                            currentHandJsonVector.handsR[i].position = Vector3.Lerp(
                                currentHandJsonVector.handsR[i].position,
                                currentHandJsonVectorNew.handsR[i].position,
                                timer / nextFrameTime);
                        }
                    if (currentHandJsonVector.handsL.Length == currentHandJsonVectorNew.handsL.Length)
                        //for each bone position in the current frame
                        for (int i = 0; i < currentHandJsonVector.handsL.Length; i++)
                        {
                            currentHandJsonVector.handsL[i].position = Vector3.Lerp(
                                currentHandJsonVector.handsL[i].position,
                                currentHandJsonVectorNew.handsL[i].position,
                                timer / nextFrameTime);
                        }
                }

                handPose.Predict3DPose(currentHandJsonVector);
            }

            //-----Facial Mocap------ -
            if (currentFaceJson != null && enableFace)
            {
                facialExpressionHandler.UpdateData(currentFaceJson);
            }

            //character.transform.rotation = characterRotation;
            slider.value = currentAnimationSlot;
        }
        catch (Exception e)
        {
            slider.value = currentAnimationSlot;
            //character.transform.rotation = characterRotation;
            Console.WriteLine(e);
            throw;
        }


    }



    #endregion


    #region Cliff



    private void CliffStartMethod()
    {
        //Json Read



        string filePath = Path.Combine(Application.dataPath, CliffConfiguration.filePath);

        if (File.Exists(filePath))
        {
            // Dosyanın içeriğini okuyun
            string jsonText = File.ReadAllText(filePath);



            //JSON verisini işleyin(örneğin, Debug.Log ile konsola yazdırın)
            //Debug.Log("JSON Verisi: " + jsonText);

            // JSON verisini parse et
            CliffPoseJson data = JsonConvert.DeserializeObject<CliffPoseJson>(jsonText);

            // Parse edilen veriyi kullan
            //Debug.Log("Name: " + data.frames[0].BodyPartRotations[0].x);

            cliffFrames = data.frames;
            cliffIsActive = true;


        }
        else
        {
            Debug.LogError("JSON dosyası bulunamadı: " + filePath);
        }

    }

    private bool cliffIsActive = true;

    private float cliffTimer = 0;
    private int currentCliffAnimationIndex = 0;
    private float cliffNextFrameTime = 0.0833f;

    private List<CliffFrame> cliffFrames;
    private CliffFrame currentCliffFrameData;


    public CliffFrame currentCliffPose;
    public CliffFrame currentCliffPoseNew;

    private void Cliff_FixedUpdate()
    {
        if (!cliffIsActive)
            return;

        cliffTimer += Time.fixedDeltaTime;

        currentCliffFrameData = cliffFrames[currentCliffAnimationIndex];

        if (cliffTimer >= cliffNextFrameTime)
        {
            Debug.Log("Cliff_FixedUpdate");

            timer = 0;
            currentCliffAnimationIndex++;

            pose3DMapper.PredictCliff3DPose(currentCliffFrameData);
            handPose.PredictCliff3DPose(currentCliffFrameData);

        }

    }

    #endregion


    #region FixedUpdate Content

    private void PlayVideo()
    {
        if (!videoPlayer.isPaused &&
            Mathf.Abs((float)(videoPlayer.time - (currentFaceJson.time / 1000.0f))) > 0.2f &&
            currentFaceJson.time != 0.0f)
        {
            videoPlayer.Pause();
        }
        else if (videoPlayer.isPaused)
        {
            videoPlayer.Play();
        }
    }

    private void DebugMethod()
    {
        if (timer > nextFrameTime)
        {
            timer = 0;
            if (!onlyCurrentIndex)
                fileIndex += 1;
        }
        try
        {
            //TestFromFile();
        }
        catch (Exception e)
        {
            print("File problem or empty array!" + "\n" + e.StackTrace);
            throw;
            Console.Write(e);
        }
    }

    private void OnAnimationPlayFinish()
    {
        videoPlayer.Pause();
        if (recording)
            StopRecording();
    }

    #endregion


    #region GetBodyParts

    private T GetBodyParts<T>(string jsonText)
    {
        return JsonUtility.FromJson<T>(jsonText);
    }
    private PoseJsonVector GetBodyPartsVector(PoseJson poseJson)
    {
        int len = poseJson.predictions.Length;
        PoseJsonVector poseJsonVector = new PoseJsonVector();
        poseJsonVector.predictions = new BodyPartVector[len];
        poseJsonVector.frame = poseJson.frame;
        poseJsonVector.width = poseJson.width;
        poseJsonVector.height = poseJson.height;
        for (int i = 0; i < len; i++)
        {
            poseJsonVector.predictions[i].position = fraction * new Vector3(-poseJson.predictions[i].x * fractionX * videoFractionX,
                -poseJson.predictions[i].y * fractionY * videoFractionY, poseJson.predictions[i].z * fractionZ * videoFractionZ);
            poseJsonVector.predictions[i].visibility = poseJson.predictions[i].visibility;

        }

        return poseJsonVector;
    }

    private HandJsonVector GetHandsVector(HandJson handJson)
    {
        int len = handJson.handsR.Length;
        int len2 = handJson.handsL.Length;
        HandJsonVector handJsonVector = new HandJsonVector();
        handJsonVector.handsR = new BodyPartVector[len];
        handJsonVector.handsL = new BodyPartVector[len2];
        handJsonVector.frame = handJson.frame;
        for (int i = 0; i < len; i++)
        {
            BodyPart data = handJson.handsR[i];
            handJsonVector.handsR[i].position = new Vector3(data.x * fractionX, -data.y * fractionY, -data.z * fractionZ);
            handJsonVector.handsR[i].visibility = data.visibility;
        }

        for (int i = 0; i < len2; i++)
        {
            BodyPart data = handJson.handsL[i];
            handJsonVector.handsL[i].position = new Vector3(data.x * fractionX, -data.y * fractionY, -data.z * fractionZ);
            handJsonVector.handsL[i].visibility = data.visibility;
        }
        return handJsonVector;
    }

    #endregion


    #region Set

    public void SetHandPoseList(List<HandJson> estimated)
    {
        framesLoaded = false;
        estimatedHandPose.Clear();
        currentHandJsonVector = GetHandsVector(estimated[0]);
        foreach (HandJson poseJson in estimated)
        {
            estimatedHandPose.Add(GetHandsVector(poseJson));
        }
        Debug.Log(estimatedHandPose.Count);
        Debug.Log(estimatedHandPose[estimatedHandPose.Count / 2].handsL.Length);
        Debug.Log(estimatedHandPose[estimatedHandPose.Count / 2].handsR.Length);
    }

    public void SetPoseList(List<PoseJson> estimated)
    {
        estimatedPoses.Clear();
        framesLoaded = false;
        currentPoseJsonVectorNew = GetBodyPartsVector(estimated[0]);
        foreach (PoseJson poseJson in estimated)
        {
            estimatedPoses.Add(GetBodyPartsVector(poseJson));
        }
        Debug.Log(estimated.Count);
    }

    public void SetFaceMocapList(List<FaceJson> estimated)
    {
        framesLoaded = false;
        print(estimated.Count);
        currentFaceJsonNew = estimated[0];
        estimatedFacialMocap = estimated;
    }

    #endregion


    #region Frames

    public void LoadFrames(FrameData[] frameData)
    {
        this.frameData = frameData.ToList<FrameData>();
        handPose.DataCleaner(frameData);

        Debug.Log(frameData.Length);

        MakeSceneReady();
    }

    public void ArrangeDataFrames()
    {
        frameData.Clear();
        int handFrame = 0;
        int faceFrame = 0;
        int bodyFrame = 0;
        int minFrame = 0;

        int bodyIndex = 0;
        int faceIndex = 0;
        int handIndex = 0;

        int index = 0;

        while (true)
        {
            if (bodyIndex < estimatedPoses.Count)
            {
                bodyFrame = estimatedPoses[bodyIndex].frame;
            }
            else
            {
                bodyFrame = int.MaxValue; //no more frames!
            }

            if (handIndex < estimatedHandPose.Count)
            {
                handFrame = estimatedHandPose[handIndex].frame;
            }
            else
            {
                handFrame = int.MaxValue; //no more frames!
            }

            if (faceIndex < estimatedFacialMocap.Count)
            {
                faceFrame = estimatedFacialMocap[faceIndex].frame;
            }
            else
            {
                faceFrame = int.MaxValue; //no more frames!
            }

            minFrame = Mathf.Min(bodyFrame, handFrame, faceFrame);

            if (minFrame == Int32.MaxValue)
                break;

            FrameData currentFrameData = new FrameData();

            if (bodyFrame == minFrame)
            {
                currentFrameData.poseData = estimatedPoses[bodyIndex];

                //Debug.Log($"currentFrameData - height: {currentFrameData.poseData.height}");
                //Debug.Log($"currentFrameData - width: {currentFrameData.poseData.width}");

                var predictions = currentFrameData.poseData.predictions;

                //Debug.Log($"predictions: {predictions.Length}");
                //Debug.Log($"predictions: {predictions[0].position}");

                bodyIndex++;
            }
            if (handFrame == minFrame)
            {
                currentFrameData.handData = estimatedHandPose[handIndex];
                handIndex++;
            }

            if (faceFrame == minFrame)
            {
                currentFrameData.faceData = estimatedFacialMocap[faceIndex];
                faceIndex++;
            }

            Debug.Log(minFrame);
            currentFrameData.frame = minFrame;
            frameData.Add(currentFrameData);
            index++;
        }

        MakeSceneReady();
    }

    public FrameData[] GetFrameData()
    {
        Debug.Log(frameData.ToArray().Length);
        return frameData.ToArray();
    }

    #endregion


    #region Meta

    //private void TestFromFile()
    //{
    //    if (enableFileSeriesReader)
    //    {
    //        StreamReader reader = new StreamReader(path + "" + fileIndex + ".json");
    //        jsonTest = reader.ReadToEnd();
    //    }
    //    else
    //    {
    //        if (readFromFilePose)
    //            jsonTest = jsonTestPose.text;
    //        if (readFromFileHand)
    //            jsonTest = jsonTestHand.text;
    //        if (readFromFace)
    //            jsonTest = jsonTestFace.text;
    //    }
    //    if (readFromFilePose)
    //    {
    //        currentPoseJson = GetBodyParts<PoseJson>(jsonTest);
    //        currentPoseJsonVector = GetBodyPartsVector(currentPoseJson);
    //        pose3DMapper.Predict3DPose(currentPoseJsonVector);

    //        videoPlayer.frame = currentPoseJson.frame;
    //        videoPlayer.Play();
    //        videoPlayer.Pause();
    //    }
    //    if (readFromFileHand)
    //    {
    //        HandJson handJson = GetBodyParts<HandJson>(jsonTest);
    //        HandJsonVector handsVector = GetHandsVector(handJson);
    //        handPose.Predict3DPose(handsVector);
    //        videoPlayer.frame = handJson.frame;
    //        videoPlayer.Play();
    //        videoPlayer.Pause();
    //    }

    //    if (readFromFace)
    //    {
    //        FaceJson faceJson = GetBodyParts<FaceJson>(jsonTest);
    //        facialExpressionHandler.UpdateData(faceJson);
    //        videoPlayer.frame = faceJson.frame;
    //        videoPlayer.Play();
    //        videoPlayer.Pause();
    //    }
    //}

    //set fractions based on video aspect ratio
    public void SetVideoFractions(float aspectRatio)
    {
        if (enableVideoAspectRatioEffector)
        {

            videoFractionX = 1;
            videoFractionY = aspectRatio;
            videoFractionZ = 1;
        }
    }

    public void SetNewCharacter(GameObject newCharacter)
    {
        character.SetActive(false);
        character = newCharacter;
        pose3DMapper.SetCharacter(character);
        try
        {
            enableHands = true;
            handPose.SetCharacter(character);
        }
        catch (Exception e)
        {
            enableHands = false;
            Console.WriteLine(e);
        }

        if (character.GetComponentInChildren<BlendShapeController>() != null)
        {
            facialExpressionHandler.SetCharacter(newCharacter);
            enableFace = true;
        }
        else
        {
            enableFace = false;
        }

        characterRotation = character.transform.rotation;
        HideCharacter();
    }

    private void MakeSceneReady()
    {
        framesLoaded = true;
        slider.maxValue = frameData.Count;
        slider.interactable = true;
        UIManager.Instancce.ActiveAnimationControlPanel();
        currentPoseJsonVectorNew = frameData[0].poseData;
        currentHandJsonVectorNew = frameData[0].handData;
        currentFaceJsonNew = frameData[0].faceData;
        currentAnimationSlot = 0;
        // currentAnimationSlot = frameData[0].frame;
    }

    public void HideCharacter()
    {

        character.SetActive(false);
    }

    public void ShowCharacter()
    {
        character.SetActive(true);
    }

    public void OnTogglePlay()
    {
        if (enableVideo && videoPlayer.url != "")
            nextFrameTime = 1 / videoPlayer.frameRate;

        timer = 10;
        videoPlayer.frame = 0;

        pause = !pause;
        // videoPlayer.Play();
        if (enableVideo && videoPlayer.url != "")
            test();

        // nextFrameTime = (float) (videoPlayer.length / videoPlayer.frameCount);

        // StartCoroutine(TestCo());
        // Invoke("test",1.5f);

    }

    private void test()
    {
        if (pause)
            videoPlayer.Pause();
        else
            videoPlayer.Play();
    }

    public void SetFaceOriginalVideo(string path)
    {
        if (!enableVideo)
            return;
        videoPlayer.url = path;
        videoPlayer.Prepare();
        videoPlayer.Play();
        videoPlayer.frame = 0;
        videoPlayer.Pause();
    }

    #endregion


    #region Recording

    private bool recording = false;

    [SerializeField]
    private float playSpeed;

    public void StartRecording()
    {
        if (!pause)
            return;
        UIManager.Instancce.DeActiveAnimationControlPanel();
        recorder.SetActive(true);
        recording = true;
        OnTogglePlay();
    }

    private void StopRecording()
    {
        recording = false;
        recorder.SetActive(false);
        UIManager.Instancce.ActiveAnimationControlPanel();
        UIManager.Instancce.ShowSuccessMessage("Animation Recorded successfully!");
    }

    #endregion


    #region Set Camera Zoom

    public void SetFaceZoomCamera()
    {
        camera.transform.position = faceZoomCameraPlace.position;
        camera.transform.rotation = faceZoomCameraPlace.rotation;
        recorder.transform.position = faceZoomCameraPlace.position;
        recorder.transform.rotation = faceZoomCameraPlace.rotation;
    }

    public void SetBodyZoomCamera()
    {
        camera.transform.position = bodyZoomCameraPlace.position;
        camera.transform.rotation = bodyZoomCameraPlace.rotation;
        recorder.transform.position = bodyZoomCameraPlace.position;
        recorder.transform.rotation = bodyZoomCameraPlace.rotation;
    }

    #endregion

}


#region Struct - Helper

public enum BodyPartsOfCliff
{
    Hips = 0,
    LeftUpLeg = 1,
    RightUpLeg = 2,
    Spine1 = 3,
    LeftLeg = 4,
    RightLeg = 5,
    Spine2 = 6,
    LeftFoot = 7,
    RightFoot = 8,
    Spine3 = 9,
    LeftToeBase = 10,
    RightToeBase = 11,
    Neck = 12,
    LeftShoulder = 13,
    RightShoulder = 14,
    Head = 15,
    LeftArm = 16,
    RightArm = 17,
    LeftForeArm = 18,
    RightForeArm = 19,
    LeftHand = 20,
    RightHand = 21,
    LeftHandIndex1 = 22,
    LeftHandIndex2 = 23,
    LeftHandIndex3 = 24,
    LeftHandMiddle1 = 25,
    LeftHandMiddle2 = 26,
    LeftHandMiddle3 = 27,
    LeftHandPinky1 = 28,
    LeftHandPinky2 = 29,
    LeftHandPinky3 = 30,
    LeftHandRing1 = 31,
    LeftHandRing2 = 32,
    LeftHandRing3 = 33,
    LeftHandThumb1 = 34,
    LeftHandThumb2 = 35,
    LeftHandThumb3 = 36,
    RightHandIndex1 = 37,
    RightHandIndex2 = 38,
    RightHandIndex3 = 39,
    RightHandMiddle1 = 40,
    RightHandMiddle2 = 41,
    RightHandMiddle3 = 42,
    RightHandPinky1 = 43,
    RightHandPinky2 = 44,
    RightHandPinky3 = 45,
    RightHandRing1 = 46,
    RightHandRing2 = 47,
    RightHandRing3 = 48,
    RightHandThumb1 = 49,
    RightHandThumb2 = 50,
    RightHandThumb3 = 51
}

[Serializable]
public class CliffPoseJson
{
    public List<CliffFrame> frames;
}

[Serializable]
public class CliffBodyPart
{
    public float x;
    public float y;
    public float z;
    public float w;
}

[Serializable]
public class CliffFrame
{
    public List<CliffBodyPart> BodyPartRotations;
}



[Serializable]
public struct BodyPart
{
    public float x;
    public float y;
    public float z;
    public float visibility;
}


[Serializable]
public class PoseJson
{
    public BodyPart[] predictions;
    public float width;
    public float height;
    public int frame;

}


[Serializable]
public class FullPoseJson
{
    public PoseJson bodyPose;
    public HandJson handsPose;
    public int frame;
}


[Serializable]
public class FaceJson
{
    public float[] blendShapes;
    public int frame;
    public float time;
}


[Serializable]
public class HandJson
{
    public BodyPart[] handsR;
    public BodyPart[] handsL;
    public int frame;
}


[Serializable]
public class HandJsonVector
{
    public BodyPartVector[] handsR;
    public BodyPartVector[] handsL;
    public int frame;
}


[Serializable]
public struct BodyPartVector
{
    public Vector3 position;
    public float visibility;
}


[Serializable]
public class PoseJsonVector
{
    public BodyPartVector[] predictions;
    public float width;
    public float height;
    public int frame;

}


[Serializable]
public class FrameData
{
    public PoseJsonVector poseData;
    public FaceJson faceData;
    public HandJsonVector handData;
    public int frame;
}
#endregion