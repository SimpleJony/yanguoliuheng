
#include "zf_common_headfile.h"
uint8 find_extreme_value(uint8 *array_value,uint8 num0,uint8 num1,uint8 model)
{
    uint8 i=0,temp=0,temp1=0,temp2=0,value=0;
    if(num0>num1)   //反过来的不想写，反正num0>num1就行了
    {
        temp1=num0-num1;
        temp2=num1;
        array_value+=num0;
        value=*array_value;
        if(model)
        {
            for(i=0;i<=temp;i++)
            {
                temp=*(array_value-i);
                if(temp>value)
                {
                    temp2=num0-i;value=temp;
                }
            }
        }
        else
        {
            for(i=0;i<=temp;i++)
            {
                temp=*(array_value-i);
                if(temp<value)
                {
                    temp2=num0-i;value=temp;
                }
            }
        }

    }
    return temp2;
}
//-----------------------------------------------------------------
// 函数简介 遍历跳变点
// 参数说明 array_value     数组指针
// 参数说明 num0            起始范围
// 参数说明 num1            终止范围
// 参数说明 jump_num        跳变范围
// 参数说明 model           0反向   1正向
// 返回类型 uint8
// 使用示例 find_jump_point(remote_distance,10,SEARCH_IMAGE_W-10,30,1)
//------------------------------------------------------------------------------
uint8 find_jump_point(uint8 *array_value,uint8 num0,uint8 num1,uint8 jump_num,uint8 model)
{
    int i;
    uint8 temp_jump_point=0;
    uint8 temp_data;

    if(model)
    {
        temp_jump_point=num0;
        for(i=0;i<(num0-num1);i++)
        {
            temp_data=func_abs(array_value[num0-i]-array_value[num0-i-1]);
            if(temp_data>jump_num)
            {
                temp_jump_point=(uint8)(num0-i);
                break;
            }
        }
    }
    else
    {
        temp_jump_point=num1;
        for(i=0;i<(num0-num1);i++)
        {
            temp_data=func_abs(array_value[num1+i]-array_value[num1+i+1]);
            if(temp_data>jump_num)
            {
                temp_jump_point=(uint8)(num1+i);
                break;
            }
        }
    }
    return temp_jump_point;
}
//-----------------------------------------------------------------
// 函数简介 连接两点补线
// 参数说明 array_value     数组指针
// 参数说明 num0            起始范围
// 参数说明 num1            终止范围
// 返回类型 void            
// 使用示例 connect_point(left_edge_line,73,25);
//------------------------------------------------------------------------------
void connect_point(uint8 *array_value,uint8 num0,uint8 num1)
{
    int i;
    float point_1=(float)array_value[num0];
    float point_2=(float)array_value[num1];
    float temp_slope=(point_2-point_1)/(num0-num1);
    for(i=0;i<(num0-num1);i++)
    {
        array_value[num0-i]=(uint8)(temp_slope*i)+array_value[num0];
    }
}