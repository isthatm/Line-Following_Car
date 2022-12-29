class parallax_laserping {
    public:
        parallax_laserping( DigitalInOut& input ) {
            dio = &input;
            t.reset();
        }

        float laserping(){
            float ret;
            dio->output();
            dio->write(0);
            wait_us(200);
            dio->write(1);
            wait_us(5);
            dio->write(0);
            wait_us(5);
    
            dio->input();
            while(dio->read()==0);
            t.start();
            while(dio->read()==1);
            ret = t.read();
            t.stop();
            t.reset();
            return ret;
        }

        float laserping_cm(){ return laserping()*17150.0f; }
        operator float(){ return laserping()*17150.0f; }
    private:
        Timer t;
        DigitalInOut *dio;
};
