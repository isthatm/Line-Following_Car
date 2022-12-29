class parallax_qti {
    public:
        parallax_qti( BusInOut& input ) {
            bio = &input;
        }

        int qti_pattern(){
            int ret;    //pattern
            bio->output();          //Set  pins as an output
            bio->write(0b1111);     //Charge capacitor
            wait_us(1000);

            bio->input();           //Set pins as an input
            wait_us(200);           //Discharge capacitor for 200 us. you can measure the time for yourself

            ret = bio->read();

            return ret;
        }

        operator int(){ return qti_pattern(); }
    private:
        BusInOut *bio;
};
