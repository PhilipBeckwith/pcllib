
vector<double> simpleSmooth(vector<double> data, int buffer)
{
	vector<double> smoother;
	double average;
	int size;
	size = data.size();	

	for(int i=0; i<data.size(); i++)
	{
		smoother.push_back(data[i]);
		if(smoother.size()>buffer)
		{smoother.erase(smoother.begin());}

		average=0;
		for(int j=0; j<smoother.size(); j++)
		{average+= smoother[j];}

		data[i]=average/smoother.size();
	}
	
	while(!smoother.empty())
	{
		average=0;
		for(int i=0; i<smoother.size(); i++)
		{average+=smoother[i];}
		data.push_back(average/smoother.size());
		smoother.erase(smoother.begin());
	}
	
	return data;
}

