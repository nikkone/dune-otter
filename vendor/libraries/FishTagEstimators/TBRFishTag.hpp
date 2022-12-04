#ifndef FishTag_TBRFishTag
#define FishTag_TBRFishTag
namespace FishTagEstimators
{    
    class TBRFishTag
    {
    public:
      //! Transmit Protocol.
      enum TransmitProtocolEnum
      {
        //! R256.
        TBR_R256 = 1,
        //! R04K.
        TBR_R04K = 2,
        //! R64K.
        TBR_R64K = 4,
        //! R01M.
        TBR_R01M = 5,
        //! S256.
        TBR_S256 = 6,
        //! S64K.
        TBR_S64K = 3,
        //! HS256.
        TBR_HS256 = 7,
        //! DS256.
        TBR_DS256 = 8
      };

      //! TBR serial number.
      uint32_t serial_no;
      //! UNIX Timestamp.
      uint32_t unix_timestamp;
      //! Millisecond.
      uint16_t millis;
      //! Transmit Protocol.
      uint8_t trans_protocol;
      //! Transmitter ID.
      uint32_t trans_id;
      //! Transmitter Data.
      uint16_t trans_data;
      //! Signal to Noise Ratio.
      uint8_t snr;
      //! Transmitter Detection Frequency.
      uint8_t trans_freq;
      //! Receiver Memory Address.
      uint16_t recv_mem_addr;
      //! Latitude (WGS-84).
      double lat;
      //! Longitude (WGS-84).
      double lon;
      //! North relative to fixed LL point
      double N;
      //! East relative to fixed LL point
      double E;
      //! Down relative to fixed LL point
      double D;
      double getS256Depth() {
        return 0.392 * trans_data;
      }
    };
}
#endif // FishTag_TBRFishTag