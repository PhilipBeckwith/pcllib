
class pointColors
{
	const int size=21;
	const int colors[21]{15079755,3978315,16776985,33480,16089648,9510580,4649200,15741670,13825340,16432830,32896,15122175,11169320,16775880,8388608,11206595,8421376,16766900,128,8421504,16777215};

	int index;
	int red, blue, green;	

	public:
	pointColors()
	{
		index=0;
		red=blue=green=50;
	}	
	
	float getColor()
	{
		float color = 0|colors[index];
		index+=1;
		index = index%size;
		return color;
	}
	
	float getColor(int forcedIndex)
	{
		forcedIndex=forcedIndex%size;
		float color = 0|colors[forcedIndex];
		return color;
	}
	
	float getColor(int r, int g, int b)
	{
		int rgb;
		rgb = (r<<16)|(g<<8)|b;
		float color= 0|rgb;
		return color;
	}
	float getChromaticColor()
	{
		green += 50;
		if(green > 250){
			green=50;
			blue += 50;
			if(blue > 250){
				blue = 50;
				red+=50;
				if(red>250){
					red=50;
				}
			}
		}
		return getColor(red, green, blue);
	}
};

