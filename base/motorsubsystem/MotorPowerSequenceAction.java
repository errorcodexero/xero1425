package org.xero1425.base.motorsubsystem;

public class MotorPowerSequenceAction extends MotorAction {
    
        private double [] times_ ;
        private double [] powers_ ;
        private int index_ ;
        private double index_time_ ;
    
        public MotorPowerSequenceAction(MotorSubsystem sub, double [] times, double [] powers) throws Exception {
            super(sub) ;
            
            times_ = times ;
            powers_ = powers ;
    
            if (times_.length == 0 || powers_.length == 0 || times_.length != powers_.length)
                throw new Exception("invalid arguments to Conveyor action") ;
        }
    
        @Override
        public void start() throws Exception {
            super.start();
    
            index_ = 0 ;
            setupCurrentIndex();
        }
    
        private void setupCurrentIndex() {
            index_time_ = getSubsystem().getRobot().getTime() ;
            getSubsystem().setPower(powers_[index_]);
        }
    
        @Override
        public void run() throws Exception {
            super.run();
    
            if (getSubsystem().getRobot().getTime() - index_time_ > times_[index_])
            {
                index_++ ;
                if (index_ == times_.length)
                {
                    setDone() ;
                }
                else
                {
                    setupCurrentIndex();
                }
            }
        }
    
        @Override
        public void cancel() {
            super.cancel() ;
    
            index_ = times_.length ;
            getSubsystem().setPower(0.0);
        }
    
        @Override
        public String toString(int indent) {
            return "MotorPowerSequenceAction " ;
        }
    }
    