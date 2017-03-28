#ifdef USE_ARTVA
//ARTVA PARSING
void parseARTVAFrame()
{
    static int artvaCounter = 0;
    artvavalid = false;
    if (checkHeader() && checkFooter())
    {
        artvavalid = true;
        //Leggo i dati. (Little-Endian!!)
        uint16_t dOne = 0;
        dOne = (uint16_t)((artvaframe[5] << 8) & 0xFF00);
        dOne += (byte)((artvaframe[4]) & 0xFF);
        _distanceOne = dOne;

        int16_t aOne = 0;
        aOne = (int16_t)((artvaframe[7] << 8) & 0xFF00);
        aOne += (byte)((artvaframe[6]) & 0xFF);
        _angleOne = aOne;

        uint16_t dTwo = 0;
        dTwo = (uint16_t)((artvaframe[9] << 8) & 0xFF00);
        dTwo += (byte)((artvaframe[8]) & 0xFF);
        _distanceTwo = dTwo;

        int16_t aTwo = 0;
        aTwo = (int16_t)((artvaframe[11] << 8) & 0xFF00);
        aTwo += (byte)((artvaframe[10]) & 0xFF);
        _angleTwo = aTwo;

        uint16_t dThree = 0;
        dThree = (uint16_t)((artvaframe[13] << 8) & 0xFF00);
        dThree += (byte)((artvaframe[12]) & 0xFF);
        _distanceThree = dThree;

        int16_t aThree = 0;
        aThree = (int16_t)((artvaframe[15] << 8) & 0xFF00);
        aThree += (byte)((artvaframe[14]) & 0xFF);
        _angleThree = aThree;

        uint16_t dFour = 0;
        dFour = (uint16_t)((artvaframe[17] << 8) & 0xFF00);
        dFour += (byte)((artvaframe[16]) & 0xFF);
        _distanceFour = dFour;

        int16_t aFour = 0;
        aFour = (int16_t)((artvaframe[19] << 8) & 0xFF00);
        aFour += (byte)((artvaframe[18]) & 0xFF);
        _angleFour = aFour;
        
        _transmitterDetected = (artvaframe[20]) & 0xFF;
        _frameCounter = (artvaframe[21]) & 0xFF;

    }
    //Parsing procedure

}

boolean checkHeader()
{
    return (
        artvaframe[0] == 0xDE &&
        artvaframe[1] == 0xFA &&
        artvaframe[2] == 0xBE &&
        artvaframe[3] == 0xBA
        );
}

boolean checkFooter()
{
    return (
        artvaframe[28] == 0xAB &&
        artvaframe[29] == 0xEB &&
        artvaframe[30] == 0xAF &&
        artvaframe[31] == 0xED
        );
}
//END OF ARTVA PARSING
#endif

