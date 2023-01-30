import cv2 as cv
import mediapipe as mp
import numpy as np
import time
import json

mp_drawing = mp.solutions.drawing_utils
mp_pose=mp.solutions.pose


# function to Calculate Angles
def calculate_angle(a,b,c):
    a=np.array(a) #First
    b=np.array(b) #Mid
    c=np.array(c) #End

    radians = np.arctan2(c[1]-b[1],c[0]-b[0])-np.arctan2(a[1]-b[1],a[0]-b[0])
    angle = np.abs(radians*180.0/np.pi)

    if angle > 180.0:
        angle= 360-angle

    return angle



rep = 0  #successful reps
stage=None #Bent or Relaxed leg
ok=False   #Bool variable
feedback=None  #Advice to keep the knee bent
leg=None #Which leg is near at current situation
TimeLeft=None  #Amount of time left for a successful rep
LeftSide=False #If camera facing Leg is Left
RightSide=False #If camera facing Leg is Right
MaxAngle=0
MinAngle=180
prevRep=0
prevAngle=180

ctime=0   #current time
ptime=0   #previous time when knee was bent
TotTime=0 #Total time of excercise
InitialTime=0 #Intial time before starting excercise

#Video Input *FOR LIVE INPUT WRITE cap=cv.VideoCapture(0)*
cap = cv.VideoCapture('KneeBendVideo.mp4')


#Video characteristics
width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv.CAP_PROP_FPS))



with mp_pose.Pose(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as pose:

    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            print("Ignoring empty Camera Frame")
            # If loading a video, use 'break' instead of 'continue'.
            continue


        #Recolor image
        image = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        image.flags.writeable = False
        
        #Make detection
        results = pose.process(image)
        
        #Recolor back to BGR
        image.flags.writeable=True
        image = cv.cvtColor(image,cv.COLOR_RGB2BGR)
        

        try:
            #Extract landmarks
            landmarks = results.pose_landmarks.landmark


            #The leg closer to camera will be considered
            LeftLeg=landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].z #z-landmark of LEFT HIP
            RightLeg=landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].z #z-landmark of RIGHT HIP
            if LeftLeg<RightLeg:  #The leg closer has negative z coordinate
                LeftSide=True
                RightSide=False
                leg='"LEFT LEG"'
            else:
                RightSide=True
                LeftSide=False
                leg='"RIGHT LEG"'

            #extracting position of hip, knee and ankle w.r.t the leg near camera
            if LeftSide:
                hip = [landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x,landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y]
                knee = [landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].x,landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].y]
                ankle = [landmarks[mp_pose.PoseLandmark.LEFT_ANKLE.value].x,landmarks[mp_pose.PoseLandmark.LEFT_ANKLE.value].y]
            else:
                hip = [landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x,landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].y]
                knee = [landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].x,landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].y]
                ankle = [landmarks[mp_pose.PoseLandmark.RIGHT_ANKLE.value].x,landmarks[mp_pose.PoseLandmark.RIGHT_ANKLE.value].y]



            #calculate angle
            angle = calculate_angle(hip,knee,ankle)

            #Maximum angle
            MaxAngle=max(MaxAngle,angle)

            #Minimum angle
            MinAngle=min(MinAngle,angle)


            #bent counter Logic
            ctime=time.time() #current time
            if angle>=170:    # for a successful rep we should straighten our leg hence angle should be >170
                prevRep=rep

            if angle<140: 
                stage="Bent"
            else:
                stage="Relaxed"
                ok=False
                feedback=""
                TimeLeft=""
            print(angle)  
            if rep==prevRep:   #if the prev rep was successful
                TotTime=ctime
                if InitialTime==0:
                    InitialTime=ctime
                if ok and stage=="Bent":  #if counting is allowed and the leg is bent
                    if ctime-ptime>=8:
                        rep+=1
                        ok=False
                        feedback=""
                        TimeLeft=""
                    else:
                        if angle>prevAngle:
                            feedback='"KEEP YOUR LEG BENT"'   # If angle increases we may assume that the guy is relaxing and we need to advice him to keep his leg bent
                        TimeLeft=8-(ctime-ptime) #calculate how much time is left
                        TimeLeft=round(TimeLeft,2)

            if not(ok) and stage=="Relaxed":
                    ok=True
                    ptime=ctime
                    feedback=""
                    TimeLeft=""

            if not(ok) and stage=="Bent":             #already bent the leg for 8 sec then you can relax
                    feedback='"YOU CAN RELAX"'

            prevAngle=angle  
            # for status box
            cv.rectangle(image, (0,0), (int(width), 70), (203,192,255), -1)

            # for Stage data
            cv.putText(image, 'STAGE:', (0,25), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv.LINE_AA)

            cv.putText(image, stage, (0,60), cv.FONT_HERSHEY_SIMPLEX, 1, (139,0,0), 2, cv.LINE_AA)
            
            
            # for Rep data
            cv.putText(image, 'REPS:', (140,25), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv.LINE_AA)
            
            cv.putText(image, str(rep), (150,60), cv.FONT_HERSHEY_SIMPLEX, 1, (0,100,0), 2, cv.LINE_AA)
            

            #for Feedback
            cv.putText(image,'FEEDBACK:', (320,25), cv.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1,cv.LINE_AA)

            cv.putText(image, feedback, (200,60), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv.LINE_AA)


            #more time required to bent knee for a successful rep
            cv.putText(image,'TIME LEFT:', (620,25), cv.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1,cv.LINE_AA)

            cv.putText(image, str(TimeLeft), (620,60), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv.LINE_AA)

            #which leg 
            cv.putText(image,'LEG:',(774,25),cv.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1,cv.LINE_AA)
            cv.putText(image,leg,(740,60),cv.FONT_HERSHEY_SIMPLEX,0.5,(139,0,0),1,cv.LINE_AA)

            for id,lm in enumerate (results.pose_landmarks.landmark):   #enumerate through all points
                h, w, c = image.shape 
                cx,cy = int(lm.x * w), int(lm.y * h)
                if (LeftSide and (id == 23 or id==25 or id==27)) : #For left leg hip, knee, and ankle joints
                    cv.circle(image,(cx,cy),10,(0,255,0),cv.FILLED) 
                if(RightSide and (id==24 or id==26 or id==28)):  #For right leg hip, knee, and ankle joints
                    cv.circle(image,(cx,cy),10,(0,255,0),cv.FILLED)
                        


        except:
            pass

        #display
        cv.imshow('Knee Bend Exercise Counter',image)
        

        if cv.waitKey(10) & 0xFF == ord('q'):
            break

mydict={'REPS : ':rep,'TOTAL TIME OF EXERCISE (IN MINUTES) : ':round((TotTime-InitialTime)/60.0,2),
        'Maximum Angle of stretch : ':round(180-MinAngle,2),'Minimum Angle of stretch : ':round(180-MaxAngle,2)}


with open('report.txt', 'w') as convert_file:
     convert_file.write(json.dumps(mydict))
cap.release()
cv.destroyAllWindows()
