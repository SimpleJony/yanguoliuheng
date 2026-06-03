#include "search_line.h"

uint8 reference_point = 0;
uint8 reference_col = SEARCH_IMAGE_W / 2;
uint8 white_max_point = 0;
uint8 white_min_point = 0;
uint8 reference_contrast_ratio = 20;
uint8 reference_col_line[SEARCH_IMAGE_H];
uint8 remote_diatance[SEARCH_IMAGE_W];
uint8 left_edge_line[SEARCH_IMAGE_H];
uint8 right_edge_line[SEARCH_IMAGE_H];
uint8 center_line[SEARCH_IMAGE_H];
uint32 if_count = 0;

uint8 find_extreme_value(uint8 *array_value, uint8 num0, uint8 num1, uint8 model)
{
    uint8 i = 0;
    uint8 count = 0;
    uint8 index = num0;
    uint8 value = array_value[num0];
    uint8 temp = 0;

    if(num0 > num1)
    {
        count = num0 - num1;
        for(i = 0; i <= count; i++)
        {
            temp = array_value[num0 - i];
            if((model && temp > value) || (!model && temp < value))
            {
                value = temp;
                index = num0 - i;
            }
        }
    }
    else
    {
        count = num1 - num0;
        for(i = 0; i <= count; i++)
        {
            temp = array_value[num0 + i];
            if((model && temp > value) || (!model && temp < value))
            {
                value = temp;
                index = num0 + i;
            }
        }
    }

    return index;
}

uint8 find_jump_point(uint8 *array_value, uint8 num0, uint8 num1, uint8 jump_num, uint8 model)
{
    int i;
    uint8 temp_jump_point = 0;
    uint8 temp_data = 0;

    if(num0 <= num1)
    {
        return num0;
    }

    if(model)
    {
        temp_jump_point = num0;
        for(i = 0; i < (num0 - num1); i++)
        {
            temp_data = func_abs(array_value[num0 - i] - array_value[num0 - i - 1]);
            if(temp_data > jump_num)
            {
                temp_jump_point = (uint8)(num0 - i);
                break;
            }
        }
    }
    else
    {
        temp_jump_point = num1;
        for(i = 0; i < (num0 - num1); i++)
        {
            temp_data = func_abs(array_value[num1 + i] - array_value[num1 + i + 1]);
            if(temp_data > jump_num)
            {
                temp_jump_point = (uint8)(num1 + i);
                break;
            }
        }
    }

    return temp_jump_point;
}

void connect_point(uint8 *array_value, uint8 num0, uint8 num1)
{
    int i;
    float point_1 = 0.0f;
    float point_2 = 0.0f;
    float temp_slope = 0.0f;

    if(num0 <= num1)
    {
        return;
    }

    point_1 = (float)array_value[num0];
    point_2 = (float)array_value[num1];
    temp_slope = (point_2 - point_1) / (float)(num0 - num1);

    for(i = 0; i <= (num0 - num1); i++)
    {
        array_value[num0 - i] = (uint8)(point_1 + temp_slope * (float)i);
    }
}

static int32 search_line_contrast(int16 point_1, int16 point_2)
{
    int16 denominator = point_1 + point_2;

    if(denominator <= 0)
    {
        return 0;
    }

    return (int32)(point_1 - point_2) * 200 / denominator;
}

void get_reference_point(const uint8 *image)
{
    uint16 i;
    const uint8 *p = &image[(MT9V03X_H - REFERENCEROW) * MT9V03X_W];
    uint16 temp = REFERENCEROW * SEARCH_IMAGE_W;
    uint32 sum = 0;

    for(i = 0; i < temp; i++)
    {
        sum += *(p + i);
    }

    reference_point = (uint8)(sum / temp);
    white_max_point = (uint8)func_limit_ab((int32)reference_point * WHITEMAXMUL / 10, BLACKPOINT, 255);
    white_min_point = (uint8)func_limit_ab((int32)reference_point * WHITEMINMUL / 10, BLACKPOINT, 255);
}

void search_reference_col(const uint8 *image)
{
    int col;
    int row;
    int i;
    int16 temp1 = 0;
    int16 temp2 = 0;
    int32 temp3 = 0;
    int16 prev_boundary_row = SEARCH_IMAGE_H - 1;
    int16 search_start = 0;
    int16 search_end = 0;
    uint8 found = 0;

    for(col = 0; col < SEARCH_IMAGE_W; col++)
    {
        remote_diatance[col] = SEARCH_IMAGE_H - 1;
    }

    for(col = 0; col < SEARCH_IMAGE_W; col += 2)
    {
        if(col == 0)
        {
            search_start = SEARCH_IMAGE_H - 1;
            search_end = STOPROW;
        }
        else
        {
            search_start = prev_boundary_row + REFERENCE_VERTICAL_RANGE;
            search_end = prev_boundary_row - REFERENCE_VERTICAL_RANGE;
            search_start = (int16)func_limit_ab(search_start, STOPROW, SEARCH_IMAGE_H - 1);
            search_end = (int16)func_limit_ab(search_end, STOPROW, SEARCH_IMAGE_H - 1);
        }

        found = 0;

        for(row = search_start; row >= search_end; row -= CONTRASTOFFSET)
        {
            temp1 = *(image + row * SEARCH_IMAGE_W + col);
            temp2 = *(image + (row - STOPROW) * SEARCH_IMAGE_W + col);

            if(temp2 > white_max_point)
            {
                continue;
            }
            if(temp1 < white_min_point)
            {
                break;
            }

            temp3 = search_line_contrast(temp1, temp2);
            if(temp3 > reference_contrast_ratio)
            {
                remote_diatance[col] = (uint8)row;
                found = 1;
                break;
            }
        }

        if(!found)
        {
            for(row = SEARCH_IMAGE_H - 1; row >= STOPROW; row -= CONTRASTOFFSET)
            {
                temp1 = *(image + row * SEARCH_IMAGE_W + col);
                temp2 = *(image + (row - STOPROW) * SEARCH_IMAGE_W + col);

                if(temp2 > white_max_point)
                {
                    continue;
                }
                if(temp1 < white_min_point)
                {
                    break;
                }

                temp3 = search_line_contrast(temp1, temp2);
                if(temp3 > reference_contrast_ratio)
                {
                    remote_diatance[col] = (uint8)row;
                    break;
                }
            }
        }

        prev_boundary_row = remote_diatance[col];
    }

    reference_col = find_extreme_value(remote_diatance, SEARCH_IMAGE_W - 10, 10, 0);
    reference_col = (uint8)func_limit_ab(reference_col, CONTRASTOFFSET, SEARCH_IMAGE_W - CONTRASTOFFSET);

    for(i = 0; i < SEARCH_IMAGE_H; i++)
    {
        reference_col_line[i] = reference_col;
    }
}

void search_line(const uint8 *image)
{
    const uint8 *p = image;
    uint8 row_max = SEARCH_IMAGE_H - 1;
    uint8 row_min = STOPROW;
    uint8 col_max = SEARCH_IMAGE_W - CONTRASTOFFSET;
    uint8 col_min = CONTRASTOFFSET;
    int16 leftstartcol = reference_col;
    int16 rightstartcol = reference_col;
    int16 leftendcol = col_min;
    int16 rightendcol = col_max;
    uint8 search_time = 0;
    int16 temp1 = 0;
    int16 temp2 = 0;
    int32 temp3 = 0;
    int leftstop = 0;
    int rightstop = 0;
    int stoppoint = 0;
    int col;
    int row;

    if_count = 0;

    for(row = 0; row < SEARCH_IMAGE_H; row++)
    {
        left_edge_line[row] = col_min - CONTRASTOFFSET;
        right_edge_line[row] = col_max + CONTRASTOFFSET;
    }

    for(row = row_max; row >= row_min; row--)
    {
        p = &image[row * SEARCH_IMAGE_W];

        if(!leftstop)
        {
            search_time = 2;
            do
            {
                if(search_time == 1)
                {
                    leftstartcol = reference_col;
                    leftendcol = col_min;
                }
                search_time--;

                for(col = leftstartcol; col >= leftendcol; col--)
                {
                    temp1 = *(p + col);
                    temp2 = *(p + col - CONTRASTOFFSET);

                    if(temp1 < white_min_point && col == leftstartcol && leftstartcol == reference_col)
                    {
                        leftstop = 1;
                        for(stoppoint = row; stoppoint >= 0; stoppoint--)
                        {
                            left_edge_line[stoppoint] = col_min;
                        }
                        search_time = 0;
                        break;
                    }
                    if(temp2 > white_max_point)
                    {
                        continue;
                    }
                    if(temp1 < white_min_point)
                    {
                        left_edge_line[row] = (uint8)col;
                        leftstartcol = (uint8)func_limit_ab(col + SEARCHRANGE, col, col_max);
                        leftendcol = (uint8)func_limit_ab(col - SEARCHRANGE, col_min, col);
                        search_time = 0;
                        break;
                    }

                    temp3 = search_line_contrast(temp1, temp2);
                    if_count++;
                    if(temp3 > reference_contrast_ratio || col == col_min)
                    {
                        left_edge_line[row] = (uint8)(col - CONTRASTOFFSET);
                        leftstartcol = (uint8)func_limit_ab(col + SEARCHRANGE, col, col_max);
                        leftendcol = (uint8)func_limit_ab(col - SEARCHRANGE, col_min, col);
                        search_time = 0;
                        break;
                    }
                }
            }while(search_time);
        }

        if(!rightstop)
        {
            search_time = 2;
            do
            {
                if(search_time == 1)
                {
                    rightstartcol = reference_col;
                    rightendcol = col_max;
                }
                search_time--;

                for(col = rightstartcol; col <= rightendcol; col++)
                {
                    temp1 = *(p + col);
                    temp2 = *(p + col + CONTRASTOFFSET);

                    if(temp1 < white_min_point && col == rightstartcol && rightstartcol == reference_col)
                    {
                        rightstop = 1;
                        for(stoppoint = row; stoppoint >= 0; stoppoint--)
                        {
                            right_edge_line[stoppoint] = col_max;
                        }
                        search_time = 0;
                        break;
                    }
                    if(temp2 > white_max_point)
                    {
                        continue;
                    }
                    if(temp1 < white_min_point)
                    {
                        right_edge_line[row] = (uint8)col;
                        rightstartcol = (uint8)func_limit_ab(col - SEARCHRANGE, col_min, col);
                        rightendcol = (uint8)func_limit_ab(col + SEARCHRANGE, col, col_max);
                        search_time = 0;
                        break;
                    }

                    temp3 = search_line_contrast(temp1, temp2);
                    if_count++;
                    if(temp3 > reference_contrast_ratio || col == col_max)
                    {
                        right_edge_line[row] = (uint8)(col + CONTRASTOFFSET);
                        rightstartcol = (uint8)func_limit_ab(col - SEARCHRANGE, col_min, col);
                        rightendcol = (uint8)func_limit_ab(col + SEARCHRANGE, col, col_max);
                        search_time = 0;
                        break;
                    }
                }
            }while(search_time);
        }
    }
}

void cross_analysis(void)
{
    int i;
    uint32 track_width = 0;
    uint8 start_point = 0;
    uint8 end_point = 0;
    uint8 cross_flag = 0;

    for(i = (SEARCH_IMAGE_H * 2 / 3); i > (SEARCH_IMAGE_H / 3); i--)
    {
        if(right_edge_line[i] > left_edge_line[i])
        {
            track_width += (uint32)(right_edge_line[i] - left_edge_line[i]);
        }
    }

    if(track_width > SEARCH_IMAGE_W * (SEARCH_IMAGE_H * 4 / 15))
    {
        cross_flag = 1;
    }

    if(cross_flag)
    {
        start_point = find_jump_point(left_edge_line, SEARCH_IMAGE_H - 5, 5, 30, 1);
        end_point = find_jump_point(left_edge_line, SEARCH_IMAGE_H - 5, 5, 30, 0);

        if(start_point < SEARCH_IMAGE_H - 10 && end_point > 10 && start_point > end_point)
        {
            connect_point(left_edge_line, start_point + 2, end_point - 2);
        }

        start_point = find_jump_point(right_edge_line, SEARCH_IMAGE_H - 5, 5, 30, 1);
        end_point = find_jump_point(right_edge_line, SEARCH_IMAGE_H - 5, 5, 30, 0);

        if(start_point < SEARCH_IMAGE_H - 10 && end_point > 10 && start_point > end_point)
        {
            connect_point(right_edge_line, start_point + 2, end_point - 2);
        }
    }
}

void get_center_line(void)
{
    uint8 row;
    uint16 sum;

    for(row = 0; row < SEARCH_IMAGE_H; row++)
    {
        sum = (uint16)left_edge_line[row] + (uint16)right_edge_line[row];
        center_line[row] = (uint8)(sum / 2);
    }
}

void search_line_process(const uint8 *image)
{
    get_reference_point(image);
    search_reference_col(image);
    search_line(image);
    cross_analysis();
    get_center_line();
}
