#ifndef FILTER_H
#define FILTER_H

#include <vector>

class Filter
{
public:
    Filter(std::vector<double> num, std::vector<double> den, const int ord = -1);
    double filter(double new_input);
    
private:
    std::vector<double> input;
    std::vector<double> output;
    std::vector<double> coef_num;
    std::vector<double> coef_den;
    int filter_order;
};

Filter::Filter(std::vector<double> num, std::vector<double> den, const int ord)
{
    if (ord == -1)
    {
        filter_order = std::max(num.size(), den.size()) - 1;
    }
    else
    {
        if (ord > 0)
        {
            filter_order = ord;
        }
        else
        {
            std::cout << "Please enter a correct filter order\n";
        }
    }
    
    coef_num = num;
    coef_den = den;
    
    while(coef_den.size() < filter_order + 1)
    {
        coef_den.push_back(0.);
    }
    
    while(coef_num.size() < filter_order + 1)
    {
        coef_den.push_back(0.);
    }
    
    std::cout << "filter :\n" << coef_num[0];
    for (int i = 1; i < filter_order + 1; i++)
    {
        std::cout << " + " << coef_num[i] << "z^-" << i;
    }
    std::cout << "\n------------------------------\n";
    std::cout << coef_den[0];
    for (int i = 1; i < filter_order + 1; i++)
    {
        std::cout << " + " << coef_den[i] << "z^-" << i;
    }
    std::cout << '\n';
    
    // input/output intial values
    for (int i = 0; i < filter_order + 1; i++)
    {
    	input.push_back(0.);	
    	output.push_back(0.);
    }
}

double Filter::filter(double new_input)
{
    std::rotate(input.begin(), input.begin() + input.size() - 1, input.end());
    std::rotate(output.begin(), output.begin() + output.size() - 1, output.end());
    
    input[0] = new_input;
    
    output[0] = coef_num[0]*input[0];    
    for(int i = 1; i < filter_order + 1; i++)
    {
        output[0] += coef_num[i]*input[i] - coef_den[i]*output[i]; 
    }    
    //output[0] /= coef_den[0]; // 1st coef den should be 1

    return output[0];
}

#endif
