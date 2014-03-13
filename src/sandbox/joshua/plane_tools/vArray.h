#pragma once
template<class T>
 class Array3D {
       
	int m_x;
	int m_y;
	int m_z;
	T* m_array;
 
 public:
		Array3D() 
		{ 
		m_array = NULL; 
		m_x = 0; 
		m_y = 0; 
		m_z = 0; 
		}
 
		Array3D(const int x, const int y, const int z) { 
			m_array = NULL;
			initialize(x, y, z); } 
 
		~Array3D() {
			if (m_array)
			{
				delete [] m_array;
			}
		}
 
       void initialize(const int x, const int y, const int z)
		{
			if (m_array)
			{
				delete [] m_array;
			}
			m_array = new T[x*y*z];		
			m_x = x;
			m_y = y;
			m_z = z;
		}

		//AColor getRGBA(int x, int y, int z)
		//{
		//	AColor res;
		//	T t = getXYZ(x,y,z,t);
		//	if (sizeof(T)==2)
		//	{
		//		float tmp = (float)t/65535.0f;
		//		res = AColor(tmp,tmp,tmp);
		//	}
		//	else if (sizeof(T)==1)
		//	{
		//		float tmp = (float)t/255.0f;
		//		res = AColor(tmp,tmp,tmp);
		//	}
		//	else
		//		res = (AColor)t;
		//	return res;
		//}

		//float getFloat(int x, int y, int z)
		//{
		//	float res;
		//	T t = getXYZ(x,y,z,t);
		//	//if (sizeof(T==2)
		//	//{
		//	//	float tmp = float(t)/65535.0f;
		//	//	res = AColor(tmp,tmp,tmp);
		//	//}
		//	//else if (sizeof(T==1)
		//	//{
		//	//	float tmp = float(t)/255.0f;
		//	//	res = AColor(tmp,tmp,tmp);
		//	//}
		//	//else
		//		res = (float)t;
		//	return res;
		//}


		T getXYZ(int x, int y, int z)
		{
			if (x < 0) x = 0;
			else if (x >= m_x) x = m_x - 1;
			if (y < 0) y = 0;
			else if (y >= m_y) y = m_y - 1;
			if (z < 0) z = 0;
			else if (z >= m_z) z = m_z - 1;

			int index = (z * m_y * m_x) + (y * m_x) + x;
			return m_array[index];
		}
 
		T* getPtrXYZ(int x, int y, int z)
		{
			if (x < 0) x = 0;
			else if (x >= m_x) x = m_x - 1;
			if (y < 0) y = 0;
			else if (y >= m_y) y = m_y - 1;
			if (z < 0) z = 0;
			else if (z >= m_z) z = m_z - 1;
			int index = (z * m_y * m_x) + (y * m_x) + x;
			return &(m_array[index]);
		}
 
		void setXYZ(int x, int y, int z, const T data)
		{
			if (x < 0) x = 0;
			else if (x >= m_x) x = m_x - 1;
			if (y < 0) y = 0;
			else if (y >= m_y) y = m_y - 1;
			if (z < 0) z = 0;
			else if (z >= m_z) z = m_z - 1;
			int index = (z * m_y * m_x) + (y * m_x) + x;
			m_array[index] = data;
		}

		void getDimensions(int& x, int& y, int& z) const 
		{
			x = m_x;
			y = m_y;
			z = m_z;
		}
 
		int getDimX() const { return m_x; }
		int getDimY() const { return m_y; }
		int getDimZ() const { return m_z; }
		
		T* getArray()
		{
			return m_array;
		}
		
		//Array2D<T>* getPlate(const int z) const
		//{

		//	Array2D<T>* plate = new Array2D<T>; //todo: check to see if this deletes the data
		//	plate.m_x = m_x;
		//	plate.m_y = m_y;
		//	plate.m_array = getPtrXYZ(0,0,z);
		//	return &plate;
		//}
};

template<class T>
 class Array4D {
       
	size_t m_x;
	size_t m_y;
	size_t m_z;
	size_t m_t;
	T* m_array;
 
 public:
		Array4D() 
		{ 
		m_array = NULL; 
		m_x = 0; 
		m_y = 0; 
		m_z = 0; 
		m_t = 0;
		}
 
		Array4D(const size_t x, const size_t y, const size_t z, const size_t t) { 
			m_array = NULL;
			initialize(x, y, z, t); } 
 
		~Array4D() {
			if (m_array)
			{
				delete [] m_array;
			}
		}
 
       int initialize(const size_t x, const size_t y, const size_t z, const size_t t)
		{
			//DebugPrint("Array4d<%d bytes>::initialize(%d,%d,%d,%d)",sizeof(T),x,y,z,t );
			int res = 1;
			if (m_array)
			{
				delete [] m_array;
				m_array = NULL;
			}

			if (	(x>0) &&
					(y>0) && 
					(z>0) && 
					(t>0))	{
								//DebugPrint("!" );
							} else {
								
								//DebugPrint(" CLEAR END\n");
								return 0;
							}//x=y=z=t=1};
			try
			{
				m_array = new T[
								(__int64)x *
								(__int64)y *
								(__int64)z *
								(__int64)t];
			}
			catch (std::bad_alloc& ba)
			{
				ba.what();
				LPCTSTR str, title;
				str = _T("Memory Allocation Failed");
				title = _T("Volume Manager Assert");
				MessageBox(NULL,str,title,MB_OK);
				res = 0;
			}
			DbgAssert(m_array);
			m_x = x;
			m_y = y;
			m_z = z;
			m_t = t;
			DebugPrint(_M("\n"));
			return res;
		}

		//AColor getRGBA(int x, int y, int z)
		//{
		//	AColor res;
		//	T t = getXYZ(x,y,z);
		//	if (sizeof(T)==2)
		//	{
		//		float tmp = (float)t/65535.0f;
		//		res = AColor(tmp,tmp,tmp);
		//	}
		//	else if (sizeof(T)==1)
		//	{
		//		float tmp = (float)t/255.0f;
		//		res = AColor(tmp,tmp,tmp);
		//	}
		//	else
		//		res = (AColor)t;
		//	return res;
		//}

		//float getFloat(int x, int y, int z)
		//{
		//	float res;
		//	T t = getXYZ(x,y,z);
		//	//if (sizeof(T==2)
		//	//{
		//	//	float tmp = float(t)/65535.0f;
		//	//	res = AColor(tmp,tmp,tmp);
		//	//}
		//	//else if (sizeof(T==1)
		//	//{
		//	//	float tmp = float(t)/255.0f;
		//	//	res = AColor(tmp,tmp,tmp);
		//	//}
		//	//else
		//		res = (float)t;
		//	return res;
		//}


		T getXYZ(size_t x, size_t y, size_t z, size_t t)
		{
			if (x < 0) x = 0;
			else if (x >= m_x) x = m_x - 1;
			if (y < 0) y = 0;
			else if (y >= m_y) y = m_y - 1;
			if (z < 0) z = 0;
			else if (z >= m_z) z = m_z - 1;
			if (t < 0) t = 0;
			else if (t >= m_t) t = m_t - 1;

			size_t index = (t * m_z * m_y * m_x ) + (z * m_y * m_x) + (y * m_x) + x;
			return m_array[index];
		}
 
		T* getPtrXYZ(size_t x, size_t y, size_t z, size_t t)
		{
			if (x < 0) x = 0;
			else if (x >= m_x) x = m_x - 1;
			if (y < 0) y = 0;
			else if (y >= m_y) y = m_y - 1;
			if (z < 0) z = 0;
			else if (z >= m_z) z = m_z - 1;
			if (t < 0) t = 0;
			else if (t >= m_t) t = m_t - 1;

			size_t index = (t * m_z * m_y * m_x ) + (z * m_y * m_x) + (y * m_x) + x;
			return &(m_array[index]);
		}
 
		void setXYZ(size_t x, size_t y, size_t z, size_t t, const T data)
		{
			if (x < 0) x = 0;
			else if (x >= m_x) x = m_x - 1;
			if (y < 0) y = 0;
			else if (y >= m_y) y = m_y - 1;
			if (z < 0) z = 0;
			else if (z >= m_z) z = m_z - 1;
			if (t < 0) t = 0;
			else if (t >= m_t) t = m_t - 1;

			size_t index = (t * m_z * m_y * m_x ) + (z * m_y * m_x) + (y * m_x) + x;
			m_array[index] = data;
		}

		void getDimensions(size_t& x, size_t& y, size_t& z, size_t& t) const 
		{
			x = m_x;
			y = m_y;
			z = m_z;
			t = m_t;
		}
 
		size_t getDimX() const { return m_x; }
		size_t getDimY() const { return m_y; }
		size_t getDimZ() const { return m_z; }
		size_t getDimT() const { return m_t; }		

		T* getArray()
		{
			return m_array;
		}
		
		//Array2D<T>* getPlate(const int z) const
		//{

		//	Array2D<T>* plate = new Array2D<T>; //todo: check to see if this deletes the data
		//	plate.m_x = m_x;
		//	plate.m_y = m_y;
		//	plate.m_array = getPtrXYZ(0,0,z);
		//	return &plate;
		//}
};
