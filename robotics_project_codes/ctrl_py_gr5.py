
class PyControl:
   
    # constructor call at initialization
    def __init__(self, inputs):
        '''
        print('Python controller created for Gr5')

        outputs = { "wheel_commands": (0., 0.)}
        outputs = { "tower_command":  0.}
        self.timeOld = inputs["t"]
        self.intErrorR = 0.
        self.intErrorL = 0.
        self.errorRold = 0.
        self.errorLold = 0.
        '''
    
    # loop function
    def loop_PyController(self, args):
         '''
        # Retrieve the inputs
        omegaRref = 1
        omegaLref = 1
        rho = 14.0
        KP=1.53460309674929
        KI= 1.03251773221888
        KPHI = 0.026
        
        inputs = args[0]
        t = inputs["t"]
        print t
        r_wheel_speed = inputs["r_wheel_speed"]
        l_wheel_speed = inputs["l_wheel_speed"]
        omegaRrefMotor = rho * omegaRref
        omegaLrefMotor = rho * omegaLref
        if inputs["t"]>0:
            errorR = omegaRrefMotor - r_wheel_speed
            errorL = omegaLrefMotor - l_wheel_speed
            dt = t - self.timeOld
            self.intErrorR = self.intErrorR+ ((errorR + self.errorRold )* dt/2.0)
            self.intErrorL = self.intErrorL+ ((errorL + self.errorLold )* dt/2.0)
            ##-----------------PI Controller-----------##
            commandR = KP* errorR + KI* self.intErrorR
            commandL = KP* errorL + KI* self.intErrorL
            wcR = commandR + KPHI* rho * r_wheel_speed
            wcL = commandL + KPHI* rho * l_wheel_speed
            outputs = { "wheel_commands": (wcR, wcL)}## add limiter
            ##-----------------------------------------##
            self.errorRold = errorR
            self.errorLold = errorL
            self.timeOld = t
        else:
            print ( "else" )
            outputs = { "wheel_commands": (0., 0.)}
            
            
            

        return outputs
'''    
    # closing function    
    def close_PyController(self):
         '''
        print('Closing Python controller')
'''
