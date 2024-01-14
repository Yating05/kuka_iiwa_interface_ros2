/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __victor_lcm_interface_control_mode_hpp__
#define __victor_lcm_interface_control_mode_hpp__

namespace victor_lcm_interface {

class control_mode {
 public:
  int8_t mode;

 public:
  static constexpr int8_t IS_POSITION_MOTION = 0;
  static constexpr int8_t IS_CARTESIAN_MOTION = 1;
  static constexpr int8_t IS_IMPEDANCE_CONTROL = 2;
  static constexpr int8_t JOINT_POSITION = 0;
  static constexpr int8_t JOINT_IMPEDANCE = 2;
  static constexpr int8_t CARTESIAN_POSE = 1;
  static constexpr int8_t CARTESIAN_IMPEDANCE = 3;

 public:
  /**
   * Encode a message into binary form.
   *
   * @param buf The output buffer.
   * @param offset Encoding starts at thie byte offset into @p buf.
   * @param maxlen Maximum number of bytes to write.  This should generally be
   *  equal to getEncodedSize().
   * @return The number of bytes encoded, or <0 on error.
   */
  inline int encode(void *buf, int offset, int maxlen) const;

  /**
   * Check how many bytes are required to encode this message.
   */
  inline int getEncodedSize() const;

  /**
   * Decode a message from binary form into this instance.
   *
   * @param buf The buffer containing the encoded message.
   * @param offset The byte offset into @p buf where the encoded message starts.
   * @param maxlen The maximum number of bytes to reqad while decoding.
   * @return The number of bytes decoded, or <0 if an error occured.
   */
  inline int decode(const void *buf, int offset, int maxlen);

  /**
   * Retrieve the 64-bit fingerprint identifying the structure of the message.
   * Note that the fingerprint is the same for all instances of the same
   * message type, and is a fingerprint on the message type definition, not on
   * the message contents.
   */
  inline static int64_t getHash();

  /**
   * Returns "control_mode"
   */
  inline static const char *getTypeName();

  // LCM support functions. Users should not call these
  inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
  inline int _getEncodedSizeNoHash() const;
  inline int _decodeNoHash(const void *buf, int offset, int maxlen);
  inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int control_mode::encode(void *buf, int offset, int maxlen) const {
  int pos = 0, tlen;
  int64_t hash = (int64_t)getHash();

  tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  return pos;
}

int control_mode::decode(const void *buf, int offset, int maxlen) {
  int pos = 0, thislen;

  int64_t msg_hash;
  thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
  if (thislen < 0)
    return thislen;
  else
    pos += thislen;
  if (msg_hash != getHash()) return -1;

  thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
  if (thislen < 0)
    return thislen;
  else
    pos += thislen;

  return pos;
}

int control_mode::getEncodedSize() const { return 8 + _getEncodedSizeNoHash(); }

int64_t control_mode::getHash() {
  static int64_t hash = _computeHash(NULL);
  return hash;
}

const char *control_mode::getTypeName() { return "control_mode"; }

int control_mode::_encodeNoHash(void *buf, int offset, int maxlen) const {
  int pos = 0, tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->mode, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  return pos;
}

int control_mode::_decodeNoHash(const void *buf, int offset, int maxlen) {
  int pos = 0, tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->mode, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  return pos;
}

int control_mode::_getEncodedSizeNoHash() const {
  int enc_size = 0;
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  return enc_size;
}

uint64_t control_mode::_computeHash(const __lcm_hash_ptr *) {
  uint64_t hash = 0x90e9a182ecc1c212LL;
  return (hash << 1) + ((hash >> 63) & 1);
}

}  // namespace victor_lcm_interface

#endif
