#ifndef IRL_GRID_MAP_HPP
#define IRL_GRID_MAP_HPP
#include <math.h>
#include <cstdlib>
#include <stdexcept>
#include <string>

template<class T>
class IRLGridMap
{
public:
	IRLGridMap(float res, float xDim, float yDim);
	T& atPos(float xPos, float yPos);
	T& atIndex(size_t xIndex, size_t yIndex);
	~IRLGridMap();
private:
	T** array_;
	size_t xSize_;
	size_t ySize_;
	std::string exceptionString_;
	float mapRes;
};

template<class T>
IRLGridMap<T>::IRLGridMap(float res, float xDim, float yDim)
{
	mapRes = res;
	xSize_ = (size_t)ceil(xDim/res);
	ySize_ = (size_t)ceil(yDim/res);
	array_ = new T*[xSize_];
	for(int i=0; i<xSize_; i++)
	{
		array_[i] = new T[ySize_];
	}
}

template<class T>
T& IRLGridMap<T>::atPos(float xPos, float yPos)
{
	size_t xIndex = (size_t)ceil(xPos/mapRes) - 1;
	size_t yIndex = (size_t)ceil(yPos/mapRes) - 1;
	return atIndex(xIndex, yIndex);
}

template<class T>
T& IRLGridMap<T>::atIndex(size_t xIndex, size_t yIndex)
{
	if(xIndex<xSize_ && yIndex<ySize_)
	{
		return array_[xIndex][yIndex];
	}
	else
	{
		exceptionString_ = "IRLGridMap: tried to access outside array bounds. Index [" + std::to_string(xIndex) + "," + std::to_string(yIndex) + "]";
		throw std::out_of_range(exceptionString_.c_str());
	}
}

template<class T>
IRLGridMap<T>::~IRLGridMap()
{
	for(int i=0; i<xSize_; i++)
	{
		delete[] array_[i];
	}
	delete[] array_;
}

#endif // IRL_GRID_MAP_HPP
