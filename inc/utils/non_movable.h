/**
 * @brief   This is file realise NonMovable.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NON_MOVABLE_H
#define __NON_MOVABLE_H

#ifdef __cplusplus

template<class T> class NonMovable
{
  public:
	NonMovable(const NonMovable&&) = delete;
	T& operator=(T&&) = delete;

  protected:
	NonMovable() = default;
	~NonMovable() = default;
};

extern "C" {
}

#endif //__cplusplus

#endif // __NON_MOVABLE_H
