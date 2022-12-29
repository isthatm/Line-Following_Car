class parallax_ping {
    public:
        parallax_ping( DigitalInOut& input ) {
            dio = &input;
            t.reset();
        }

        float ping(){
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

        float ping_cm(){ return ping()*17700.4f; }
        operator float(){ return ping()*17700.4f; }
    private:
        Timer t;
        DigitalInOut *dio;
};
