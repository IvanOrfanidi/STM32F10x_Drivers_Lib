/**
 * @brief   This is file realise NonCopyable.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NON_COPYABLE_H
#define __NON_COPYABLE_H

#ifdef __cplusplus

template<class T> class NonCopyable
{
  public:
	NonCopyable(const NonCopyable&) = delete;
	T& operator=(const T&) = delete;

  protected:
	NonCopyable() = default;
	~NonCopyable() = default;
};

extern "C" {
}

#endif // __cplusplus

#endif // __NON_COPYABLE_H
