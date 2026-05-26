#include "zf_common_headfile.h"
#include "user_function.h"
#include "search_line.h"
 uint8 reference_point;   //参考点
 uint8 reference_col;     //参考列
 uint8 white_max_point;   //白点最大
 uint8 white_min_point;   //白点最小
 uint8 reference_contrast_ratio;//参考对比度
 uint8 reference_col_line[SEARCH_IMAGE_H];//参考列数组
 uint8 remote_diatance[SEARCH_IMAGE_W];//白点远端距离
 uint8 left_edge_line[SEARCH_IMAGE_H];//左右边界
 uint8 right_edge_line[SEARCH_IMAGE_H];
 uint32 if_count;
//获取参考点位

void get_reference_point(const uint8 *image)
{
		uint16 i;
    uint8 *p=(uint8 *)&image[(MT9V03X_H-REFERENCEROW)*MT9V03X_W];
    uint16 temp=0;//数量
    uint32 temp1=0;//统计点加起来的和
    temp = REFERENCEROW*SEARCH_IMAGE_W;//统计点的数量
    for(i=0;i<temp;i++)
    {
        temp1+=*(p+i);
    }
    reference_point=(uint8)(temp1/temp);
    white_max_point=(uint8)func_limit_ab((int32)reference_point*WHITEMAXMUL/10,BLACKPOINT,255);//必为白点值
    white_min_point=(uint8)func_limit_ab((int32)reference_point*WHITEMINMUL/10,BLACKPOINT,255);//必为黑点值
}
//搜索图像参考列
void search_reference_col(const uint8 *image)
{
    int col,row;
	int i;
    int16 temp1=0,temp2=0,temp3=0;
    for(col=0;col<SEARCH_IMAGE_W;col+=CONTRASTOFFSET)
    {
        remote_diatance[col]=SEARCH_IMAGE_H-1;
    }
    for(col=0;col<SEARCH_IMAGE_W;col+=2)
    {
        for(row=SEARCH_IMAGE_H-1;row>STOPROW;row-=CONTRASTOFFSET)
        {
            temp1=*(image+row*SEARCH_IMAGE_W+col);
            temp2=*(image+(row-STOPROW)*SEARCH_IMAGE_W+col);
            if(temp2>white_max_point)       //若对比点都是白点，则可以继续往下找
            continue;
            if(temp1<white_min_point)
            break;
            temp3=(temp1-temp2)*200/(temp1+temp2);
            if(temp3>reference_contrast_ratio||row==STOPROW)
            {
                remote_diatance[col]=(uint8)row;//row越小说明越远
                break;
            }
        }
    }
    reference_col=find_extreme_value(remote_diatance,SEARCH_IMAGE_W-10,10,0);
    reference_col=(uint8)func_limit_ab(reference_col,1,SEARCH_IMAGE_W-2);
    for(i=0;i<SEARCH_IMAGE_H;i++)
    {
        reference_col_line[i]=reference_col;
    }
}
//搜索赛道边线
void search_line(const uint8 *image)
{
    uint8*p=(uint8 *)image[0];                  //指针
    uint8 row_max=SEARCH_IMAGE_H-1;             //行最大值
    uint8 row_min=STOPROW;                      //行最小值
    uint8 col_max=SEARCH_IMAGE_W-CONTRASTOFFSET;//列最大值
    uint8 col_min=CONTRASTOFFSET;               //列最小值
    int16 leftstartcol=reference_col;           //左起始列
    int16 rightstartcol=reference_col;          //右起始列
    int16 leftendcol=0;
    int16 rightendcol=SEARCH_IMAGE_W-1;
    uint8 search_time=0;
    uint8 temp1=0,temp2=0;
    int temp3=0;
    int leftstop=0,rightstop=0,stoppoint=0;

    int col,row;

    for(row=row_max;row>=row_min;row--)
    {
        left_edge_line[row]=col_min-CONTRASTOFFSET;
        right_edge_line[row]=col_max+CONTRASTOFFSET;
    }

    for(row=row_max;row>=row_min;row--)
    {
        p=(uint8 *)&image[row*SEARCH_IMAGE_W];
        if(!leftstop)
        {
            search_time=2;
            do{
                if(search_time==1)
                {
                    leftstartcol=reference_col;
                    leftendcol=col_min;
                }
                search_time--;
                for(col=leftstartcol;col>=leftendcol;col--)
                {
                    temp1=*(p+col);

                    temp2=*(p+col-CONTRASTOFFSET);

                    if(temp1<white_min_point&&col==leftstartcol&&leftstartcol==reference_col)
                    {
                        leftstop=1;
                        for(stoppoint =row;stoppoint>=0;stoppoint--)
                        {
                            left_edge_line[stoppoint]=col_min;

                        }
                        search_time=0;
                        break;
                    }
                    if(temp2>white_max_point)       //若对比点都是白点，则可以继续往下找
                    continue;
                    if(temp1<white_min_point)
                    {
                        left_edge_line[row]=(uint8)col;
                        break;
                    }
                    temp3=(temp1-temp2)*200/(temp1+temp2);
                    if_count++;
                    if(temp3>reference_contrast_ratio||col==col_min)
                    {
                        left_edge_line[row]=col-CONTRASTOFFSET;
                        leftstartcol=(uint8)func_limit_ab(col+SEARCHRANGE,col,col_max);
                        leftendcol=(uint8)func_limit_ab(col-SEARCHRANGE,col_min,col);
                        break;
                    }

                }
            }while(search_time);
        }
        if(!rightstop)
        {
            search_time=2;
            do{
                if(search_time==1)
                {
                    rightstartcol=reference_col;
                    rightendcol=col_max;
                }
                search_time--;
                for(col=rightstartcol;col<=rightendcol;col++)
                {
                    temp1=*(p+col);

                    temp2=*(p+col+CONTRASTOFFSET);

                    if(temp1<white_min_point&&col==rightstartcol&&rightstartcol==reference_col)
                    {
                        rightstop=1;
                        for(stoppoint =row;stoppoint>=0;stoppoint--)
                        {
                            right_edge_line[stoppoint]=col_max;
                        }
                        search_time=0;
                        break;
                    }
                    if(temp2>white_max_point)       //若对比点都是白点，则可以继续往下找
                    continue;
                    if(temp1<white_min_point)
                    {
                        right_edge_line[row]=(uint8)col;
                        break;
                    }
                    temp3=(temp1-temp2)*200/(temp1+temp2);
                    if_count++;
                    if(temp3>reference_contrast_ratio||col==col_max)
                    {
                        right_edge_line[row]=col+CONTRASTOFFSET;
                        rightstartcol=(uint8)func_limit_ab(col-SEARCHRANGE,col_min,col);
                        rightendcol=(uint8)func_limit_ab(col+SEARCHRANGE,col,col_max);
                        break;
                    }

                }
            }while(search_time);
        }
    }
}
//--------------------------------------------
// 函数类型     十字检测函数
// 返回类型     void  
// 使用示例     cross_analysis();
// 备注
//--------------------------------------------------
void cross_analysis()
{
    int i;
    uint32 track_width=0;
    uint8 start_point;
    uint8 end_point;
    uint8 cross_flag;

    for(i=(MT9V03X_H*2/3);i>(MT9V03X_H/3);i--)
    {
        track_width+=(right_edge_line[i]-left_edge_line[i]);
    }
    if(track_width>MT9V03X_W*(MT9V03X_H*4/15))
    {
        cross_flag=1;
    }
    if(cross_flag==1)
    {
        start_point=find_jump_point(left_edge_line,MT9V03X_H-5,5,30,1)+2;
        end_point=find_jump_point(left_edge_line,MT9V03X_H-5,5,30,0)-2;
        connect_point(left_edge_line,start_point,end_point);

        start_point=find_jump_point(right_edge_line,MT9V03X_H-5,5,30,1)+2;
        end_point=find_jump_point(right_edge_line,MT9V03X_H-5,5,30,0)-2;
        connect_point(right_edge_line,start_point,end_point);
        if(track_width<MT9V03X_W*(MT9V03X_H*1/15))
        {
            cross_flag=0;
        }
    }
}