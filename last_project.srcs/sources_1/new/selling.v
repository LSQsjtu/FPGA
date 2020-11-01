`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/10/19 18:34:54
// Design Name: 
// Module Name: selling
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module main(key, clk, rst, a_to_g, an, sel_goods, deal, disp_dot, led);
//=========================��������===========================================
input[1:0] key; //Ͷ�Ұ�ť
input clk; //��ʱ��
input rst; //���ý��׼�¼���������ʾ
input deal; //���װ�ť
output [6:0] a_to_g; //�����������ʾ
output reg[3:0] an; //�������������
output reg disp_dot; //С����
input wire [2:0]sel_goods; // ����ѡ����Ʒ 001������� 011������ 100����������
output reg [7:0] led;//�ƹ�Ч����
//=========================����================================================
//�ѽ���Խ�Ϊ��λ
//num[15:12]��ʾ��Ʒ�۸��ʮλ
//num[11:8]��ʾ��Ʒ�۸�ĸ�λ
//num[7��4]��ʾ�������ʮλ
//num[3:0]��ʾ������ĸ�λ
reg [15:0] num = 16'b0;
reg [3:0]disp_num; //�������ʾ����
reg [11:0]num_goods = 12'h555; //��ʼ���ÿ�涼Ϊ
reg [23:0]clk_cnt = 24'b0; //ʱ�Ӽ���
reg [26:0]blink_clk_cnt = 27'b0;//���ڼ�����˸ʱ��
reg [15:0] changes; //������
reg ch_flag = 0; //�����־λ
reg goods_flag = 1; //����־λ
reg deal_flag  = 0; //�����Ƿ���еı�־λ
wire deal_success_flag; //�����Ƿ�ɹ��ı�־
reg [3:0] num_goods_disp; //������ʾ���
reg [4:0] num_5_jiao = 4'b0; //Ͷ���5�ǵ�����
reg [4:0] num_1_yuan = 4'b0; //Ͷ���1Ԫ������
reg blink_flag = 1; //��˸��־
reg blink_rst;//��˸����

//=========================��Ʒѡ��=============================================
//001������� ��Ӧ������Ϊ20
//010������ ��Ӧ������Ϊ35
//100���������� ��Ӧ������Ϊ30
always @(posedge clk)
begin
case(sel_goods)
1:num[15:8] = 8'b00100000;
2:num[15:8] = 8'b00110101;
4:num[15:8] = 8'b00110000;
default:num[15:8] = 8'b00000000;
endcase
end


//=========================������===========================================
//����5��
always @(posedge key[0] or posedge rst)
begin
if (rst)
    num_5_jiao = 4'b0;
else
    num_5_jiao = num_5_jiao + 1;
end

//����1Ԫ
always @(posedge key[1] or posedge rst) 
begin
if (rst)
    num_1_yuan = 4'b0; 
else
    num_1_yuan = num_1_yuan + 1;
end

// ���ܽ��
always @(posedge clk)
begin
    //��λ���֣�С�����֣�ֻ��5�ǵ������й�
    //5�ǵĸ���Ϊ��������Ϊ5������Ϊż������Ϊ0
    if (num_5_jiao[0])
        num[3:0] = 4'b0101;
    else 
        num[3:0] = 4'b0;
    //ʮλ��������1Ԫ��5�ǵ������й�
    //��Ϊ2��5�Ǿ���1Ԫ������ֻҪȡ5�ǵĸ�����ǰ��λ
    num[7:4] = num_1_yuan + num_5_jiao[3:1];
end


//=========================����===================================================
assign deal_success_flag = ch_flag & goods_flag;
always @(posedge deal or posedge rst)
begin
//����rst���������ý��׼�¼
if (rst)
    begin
    ch_flag = 0;
    deal_flag = 0;
    goods_flag = 1;
    blink_rst = 1;
    end
else
    begin
    blink_rst = 0;
    deal_flag = 1;
    //��������ڵ�����Ʒ�۸�                                                    
    if (num[7:0] >= num[15:8])
        begin
        changes[15:8] = 8'b0;
        //����
        if (num[3:0] >= num[11:8])
            changes[7:0] = num[7:0] - num[15:8];
        else 
            begin
            changes[3:0] = 4'b0101;
            changes[7:4] = num[7:4] - num[15:12] - 1;
            end
        
        //��־λ
        ch_flag = 1;
        //��ȥ���
        case(sel_goods)
            1:
            begin
            if (num_goods[3:0] >= 1)
                begin
                num_goods[3:0] = num_goods[3:0] - 1;
                goods_flag = 1;
                end
            else 
                goods_flag = 0;
            end
            2:
            begin
            if (num_goods[7:4] >= 1)
                begin
                num_goods[7:4] = num_goods[7:4] - 4'b1;
                goods_flag = 1;
                end
            else 
                goods_flag = 0;
            end
            4:
            begin
            if (num_goods[11:8] >= 1)
                begin
                num_goods[11:8] = num_goods[11:8] - 1;
                goods_flag = 1;
                end
            else 
                goods_flag = 0;
            end
        
        default: num_goods = num_goods;
        endcase
        end
    //������С����Ʒ�۸�
    else 
        ch_flag = 0;
    end
end

//=========================�������ʾ==============================================
always @(posedge clk)
begin
if (clk_cnt == 24'hFFFFFF)
    clk_cnt = 24'b0;
else
    clk_cnt = clk_cnt + 1;

//������˸��־
if (blink_rst)
    blink_flag = 1;

//δ���н���ʱ
if (!deal_flag)
    begin
    disp_dot = dot_get(clk_cnt, 4'b1010);
    disp_num = disp_num_get(clk_cnt, num);
    an = an_get(clk_cnt);
    end
//���׺�
else
    begin
    //���׳ɹ�����ʾ��Ǯ
    if (deal_success_flag)
        begin
        //���׳ɹ��ĵƹ���˸Ч��
        case (clk_cnt[23:21])
            0:led = 8'b00000001;
            1:led = 8'b00000010;
            2:led = 8'b00000100;
            3:led = 8'b00001000;
            4:led = 8'b00010000;
            5:led = 8'b00100000;
            6:led = 8'b01000000;
            7:led = 8'b10000000;
        endcase
        disp_dot = dot_get(clk_cnt, 4'b0010);
        disp_num = disp_num_get(clk_cnt, changes);
        an = an_get(clk_cnt);
        end
    //���㽻��ʧ�ܣ���˸����6�Σ���Լ2��
    else if (!ch_flag)
        begin
        if (blink_flag)
            begin
            blink_clk_cnt = blink_clk_cnt + 1;
            if (blink_clk_cnt[26] && blink_clk_cnt[25])
                begin
                blink_flag = 0;
                blink_clk_cnt = 0;
                end
            if (clk_cnt[23])
                begin
                disp_dot = dot_get(clk_cnt, 4'b1010);
                disp_num = disp_num_get(clk_cnt, num);
                an = an_get(clk_cnt);
                end         
            end
        else
            begin
            disp_dot = dot_get(clk_cnt, 4'b1000);
            disp_num = disp_num_get(clk_cnt, num);
            an = an_get(clk_cnt);
            end
        end
    //��治�㽻��ʧ�ܣ���ʾ��棬�϶���0��
    else if(!goods_flag)
        begin
        disp_dot = dot_get(clk_cnt, 4'b1111);
        disp_num = disp_num_get(clk_cnt, 16'b0);
        an = an_get(clk_cnt);
        end
    end
end

//=========================�õ���ʾ���ֺ���=========================================
//���ݼ��������������ʾ����
function [3:0] disp_num_get;
    input [23:0] clk_cnt;
    input [16:0] num;
    begin
    case (clk_cnt[15:14])
        0:disp_num_get = num[3:0];
        1:disp_num_get = num[7:4];
        2:disp_num_get = num[11:8];
        3:disp_num_get = num[15:12];
    endcase
    end
endfunction

//=========================С������ʾ����==========================================
function dot_get;
    input [23:0] clk_cnt;
    input [3:0] dot;
    begin
    case (clk_cnt[15:14])
        0:dot_get = dot[0];
        1:dot_get = dot[1];
        2:dot_get = dot[2];
        3:dot_get = dot[3];
    endcase
    end
endfunction
//=========================����ܽ����������=======================================
//���ݼ������������Ӧ�õ����������
function [3:0] an_get;
    input [23:0] clk_cnt;
    begin
    case (clk_cnt[15:14])
        0:an_get = 4'b0001;
        1:an_get = 4'b0010;
        2:an_get = 4'b0100;
        3:an_get = 4'b1000;
    endcase
    end
endfunction

clk_sw_7seg_sub A1( .NUM(disp_num),
.a_to_g(a_to_g));
endmodule 

//=========================�������ʾģ��===========================================
module clk_sw_7seg_sub(
input [3:0]NUM,
output reg[6:0]a_to_g
);

always @(*)
 begin
 case(NUM) //�ж�Ҫ��ʾ�����Σ���ֵ�����������Ӧ��ֵ
 0:a_to_g=7'b1111110; //��ʾ����"0"
 1:a_to_g=7'b0110000;
 2:a_to_g=7'b1101101;
 3:a_to_g=7'b1111001;
 4:a_to_g=7'b0110011;
 5:a_to_g=7'b1011011;
 6:a_to_g=7'b1011111;
 7:a_to_g=7'b1110000;
 8:a_to_g=7'b1111111;
 9:a_to_g=7'b1111011;
 'hA: a_to_g=7'b1110111;
 'hB: a_to_g=7'b0011111;
 'hC: a_to_g=7'b1001110;
 'hD: a_to_g=7'b0111101;
 'hE: a_to_g=7'b1001111;
 'hF: a_to_g=7'b1000111; //��ʾ����"F"
 default: a_to_g=7'b1111110;
 endcase
 end
endmodule
