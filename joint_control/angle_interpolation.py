'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        # get data
        (names, times, keys) = keyframes
        #get time intervall, maybe suftract starting time
        time_dif = perception.time

        #iterating through all joints
        for i in range(len(names)):
            joint_name = names[i]
            #check if we have data about a joint
            if joint_name in self.joint_names:

                #iterate over the time
                for j in range(len(times[i]) - 1):

                    #do interpolation with positions/ data p0 and p3 at time t0 and t3
                    # we have a time before data -> start at begin of recording
                    if time_dif < times[i][0] and j == 0:
                        #times
                        t0 = 0
                        t3 = times[i][0]
                        #positions
                        p0 = self.perception.joint[joint_name]
                        p3 = keys[i][0][0]
                       #interpolate position at time right in the provided data     
                    elif (times[i][j] < time_dif < times[i][j + 1] and j+1 < len(times[i])):
                        #times
                        t0 = times[i][j]
                        t3 = times[i][j+1]
                        #positions
                        p0 = keys[i][j][0]
                        p3 = keys[i][j+1][0]

                    elif j == 0:
                        #times
                        t0 = times[i][j]
                        t3 = times[i][j+1]
                        #positons
                        p0 = keys[i][j][0]
                        p3 = keys[i][j+1][0]
                        
   
                    p1 = keys[i][j][1][2] + p0
                    p2 = keys[i][j][2][2] + p3
                    
                    # t \in [0,1]
                    t = (time_dif-t0) / (t3-t0)
    
                    #put all data into the bizec interpolation and add this to the dictonary of results
                    target_joints[joint_name] = ((1 - t)**3)*p0 + (3 * (1 - t)**2)*p1*t + (3 * (1 - t))*p2*(t**2) + p3*(t**3)
                        

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
