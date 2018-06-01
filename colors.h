
class pointColors
{
	public:
	const int colors[8]{16711680,16747520,16774656,11468544,65280,65479,16122111,16777215};

	int index;
	
	pointColors(){index=7;}	
	
	float getColor()
	{
		float color = 0|colors[index];
		index+=1;
		index = index%8;
		//if(index == 8){index =0;}
		return color;
	}
	
	float getColor(int forcedIndex)
	{
		forcedIndex=forcedIndex%8;
		float color = 0|colors[forcedIndex];
		return color;
	}
};

