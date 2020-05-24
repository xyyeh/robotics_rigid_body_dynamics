/**
 * \class ExpClass
 *
 * \brief Provides an example classs
 *
 * This class is meant as an example.  It is not useful by itself
 * rather its usefulness is only a function of how much it helps
 * the reader.  It is in a sense defined by the person who reads it
 * and otherwise does not exist in any real form. Referenced from:
 * https://www-numi.fnal.gov/offline_software/srt_public_context/WebDocs/doxygen-howto.html
 *
 * \note Attempts at zen rarely work.
 * \author (last to touch it) $Author: bv $
 * \version $Revision: 1.5$
 * \date $Date: 2005/04/14 14:16:20 $
 *
 * Contact: bv@bnl.gov
 * Created on: Wed Apr 13 18:39:37 2005
 */

#pragma once

#include <string>

namespace robotics {
namespace exp_proj {

/** Data structure */
struct DataStructure {
  std::string str;
};

class ExpClass {
 private:
  /** member variable */
  uint32_t parameter_;

 public:
  ExpClass(){};
  virtual ~ExpClass(){};

  /**
   * \brief Function brief description
   * \return True if operation succeeds
   */
  bool CalcValue();

  /**
   * \brief Get parameter
   * \return Parameter
   */
  uint32_t Parameter() const { return parameter_; }

  /**
   * \brief Set parameter
   * \param[in]     param   input parameter
   */
  void SetParameter(const uint32_t &param) { parameter_ = param; }
};

}  // namespace exp_proj
}  // namespace robotics