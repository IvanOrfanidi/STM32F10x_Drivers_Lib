/**
 * @brief   This is file realize ring buffer.
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

/* Includes ------------------------------------------------------------------*/
/* Standart lib */
#include <stdint.h>

#ifdef __cplusplus

template<typename T> class RingBuffer
{
  public:
	/**
     * @param [in] size - buffer size
     */
	explicit RingBuffer(size_t size)
	  : _size(0)
	  , _tail(0)
	  , _head(0)
	  , _count(0)
	  , _buffer(nullptr)
	{
		init(size);
	}

	~RingBuffer()
	{
		::delete[] _buffer;
	}

	// Copy constructor
	RingBuffer(const RingBuffer& other)
	  : _size(other._size)
	  , _tail(other._tail)
	  , _head(other._head)
	  , _count(other._count)
	  , _buffer(::new T[other._size])
	{
		std::copy(other._buffer, (other._buffer + other._size), _buffer);
	}

	/**
	 * @brief Copy assignment operator
	 * @param other - new class RingBuffer
	 * @return RingBuffer& copy class RingBuffer
	 */
	RingBuffer& operator=(const RingBuffer& other)
	{
		if(this != &other) {
			// Free the existing resource
			::delete[] _buffer;

			_size = other._size;
			_tail = other._tail;
			_head = other._head;
			_count = other._count;
			_buffer = new T[other._size];

			std::copy(other._buffer, (other._buffer + other._size), _buffer);
		}
		return *this;
	}

	/**
	 * @brief Construct a new Ring Buffer object
	 * @param other - new class RingBuffer
	 */
	RingBuffer(RingBuffer&& other)
	{
		*this = std::move(other);
	}

	// Move assignment operator
	RingBuffer& operator=(RingBuffer&& other)
	{
		if(this != &other) {
			// Free the existing resource
			::delete[] _buffer;

			*this = std::move(other);
		}
		return *this;
	}

	/**
     * @brief Removes the next element in the buffer, reducing its size by one.
     * @retval next element
	*/
	T pop()
	{
		if(empty()) {
			return 0;
		}

		const auto data = _buffer[_tail++];
		if(_tail >= _size) {
			_tail = 0;
		}
		--_count;
		return data;
	}

	/**
     * @brief  Inserts a new element at the end of the buffer, after its current last element.
     * @param [in] data - data new element
	*/
	void push(const T& data)
	{
		_buffer[_head++] = data;
		if(_head >= _size) {
			_head = 0;
		}
		++_count;
	}

	/**
     * @brief Clear buffer
	*/
	void clear()
	{
		_tail = 0;
		_head = 0;
		_count = 0;
	}

	/**
     * @brief Returns whether the buffer is empty: i.e. whether its size is zero.
     * @retval true - buffer is empty
     */
	bool empty() const
	{
		return (0 == _count);
	}

	/**
     * @brief Returns the number of elements in the buffer.
     * @retval - number of elements
     */
	size_t size() const
	{
		return _count;
	}

	/**
     * @brief Returns status overflow buffer.
     * @retval true - buffer is overflow
     */
	bool overflow() const
	{
		return (_count >= _size);
	}

  private:
	RingBuffer() = delete;

	void init(size_t size)
	{
		_buffer = ::new T[size];
		if(nullptr != _buffer) {
			_size = size;
			clear();
		}
	}

	size_t _size;  //< buffer size
	size_t _tail;  //< buffer tail
	size_t _head;  //< buffer head
	size_t _count; //< number of elements
	T* _buffer;    //< pointer to buffer
};

extern "C" {
}
#endif // __cplusplus

#endif // __RING_BUFFER_H
