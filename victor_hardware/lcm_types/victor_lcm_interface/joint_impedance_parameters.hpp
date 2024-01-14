/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __victor_lcm_interface_joint_impedance_parameters_hpp__
#define __victor_lcm_interface_joint_impedance_parameters_hpp__

#include "victor_lcm_interface/joint_value_quantity.hpp"

namespace victor_lcm_interface {

class joint_impedance_parameters {
 public:
  victor_lcm_interface::joint_value_quantity joint_stiffness;

  victor_lcm_interface::joint_value_quantity joint_damping;

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
   * Returns "joint_impedance_parameters"
   */
  inline static const char *getTypeName();

  // LCM support functions. Users should not call these
  inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
  inline int _getEncodedSizeNoHash() const;
  inline int _decodeNoHash(const void *buf, int offset, int maxlen);
  inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int joint_impedance_parameters::encode(void *buf, int offset, int maxlen) const {
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

int joint_impedance_parameters::decode(const void *buf, int offset, int maxlen) {
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

int joint_impedance_parameters::getEncodedSize() const { return 8 + _getEncodedSizeNoHash(); }

int64_t joint_impedance_parameters::getHash() {
  static int64_t hash = _computeHash(NULL);
  return hash;
}

const char *joint_impedance_parameters::getTypeName() { return "joint_impedance_parameters"; }

int joint_impedance_parameters::_encodeNoHash(void *buf, int offset, int maxlen) const {
  int pos = 0, tlen;

  tlen = this->joint_stiffness._encodeNoHash(buf, offset + pos, maxlen - pos);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = this->joint_damping._encodeNoHash(buf, offset + pos, maxlen - pos);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  return pos;
}

int joint_impedance_parameters::_decodeNoHash(const void *buf, int offset, int maxlen) {
  int pos = 0, tlen;

  tlen = this->joint_stiffness._decodeNoHash(buf, offset + pos, maxlen - pos);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = this->joint_damping._decodeNoHash(buf, offset + pos, maxlen - pos);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  return pos;
}

int joint_impedance_parameters::_getEncodedSizeNoHash() const {
  int enc_size = 0;
  enc_size += this->joint_stiffness._getEncodedSizeNoHash();
  enc_size += this->joint_damping._getEncodedSizeNoHash();
  return enc_size;
}

uint64_t joint_impedance_parameters::_computeHash(const __lcm_hash_ptr *p) {
  const __lcm_hash_ptr *fp;
  for (fp = p; fp != NULL; fp = fp->parent)
    if (fp->v == joint_impedance_parameters::getHash) return 0;
  const __lcm_hash_ptr cp = {p, (void *)joint_impedance_parameters::getHash};

  uint64_t hash = 0x183f83ab1b5b43a7LL + victor_lcm_interface::joint_value_quantity::_computeHash(&cp) +
                  victor_lcm_interface::joint_value_quantity::_computeHash(&cp);

  return (hash << 1) + ((hash >> 63) & 1);
}

}  // namespace victor_lcm_interface

#endif
