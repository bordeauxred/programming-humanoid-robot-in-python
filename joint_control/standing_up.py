'''In this exercise you need to put all code together to make the robot be able to stand up by its own.

* Task:
    complete the `StandingUpAgent.standing_up` function, e.g. call keyframe motion corresponds to current posture

'''


from recognize_posture import PostureRecognitionAgent


class StandingUpAgent(PostureRecognitionAgent):
    def think(self, perception):
        self.standing_up()
        return super(StandingUpAgent, self).think(perception)

    def standing_up(self):
        posture = self.posture
        # check which posture we recognize and take a appropiate action
        if posture == 'Belly' or posture == 'Right':
            self.keyframes = rightBellyToStand()
        if posture == 'Back':
            self.keyframes = rightBackToStand()
        if posture == 'Crouch':
            self.keyframes = leftBellyToStand()
        if posture == 'Frog':
            self.keyframes = leftBellyToStand()
        if posture == 'Left':
            self.keyframes = rightBellyToStand()
        if posture == 'Knee':
            self.keyframes = rightBellyToStand()
        if posture == 'HeadBack':
            self.keyframes = wipe_forehead()
        if posture == 'Sit':
            self.keyframes = rightBackToStand()
        if posture == 'StandInit':
            self.keyframes = rightBackToStand()
        # else posture is Stand, do nothing and say hello
        else:
            self.keyframes = hello()
            
        


class TestStandingUpAgent(StandingUpAgent):
    '''this agent turns off all motor to falls down in fixed cycles
    '''
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(TestStandingUpAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.stiffness_on_off_time = 0
        self.stiffness_on_cycle = 10  # in seconds
        self.stiffness_off_cycle = 3  # in seconds

    def think(self, perception):
        action = super(TestStandingUpAgent, self).think(perception)
        time_now = perception.time
        if time_now - self.stiffness_on_off_time < self.stiffness_off_cycle:
            action.stiffness = {j: 0 for j in self.joint_names}  # turn off joints
        else:
            action.stiffness = {j: 1 for j in self.joint_names}  # turn on joints
        if time_now - self.stiffness_on_off_time > self.stiffness_on_cycle + self.stiffness_off_cycle:
            self.stiffness_on_off_time = time_now

        return action


if __name__ == '__main__':
    agent = TestStandingUpAgent()
    agent.run()
