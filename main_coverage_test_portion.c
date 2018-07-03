{\rtf1\ansi\ansicpg1252\cocoartf1561\cocoasubrtf400
{\fonttbl\f0\fnil\fcharset0 Monaco;}
{\colortbl;\red255\green255\blue255;\red63\green127\blue95;}
{\*\expandedcolortbl;;\csgenericrgb\c24706\c49804\c37255;}
\paperw11900\paperh16840\margl1440\margr1440\vieww10800\viewh8400\viewkind0
\deftab720
\pard\pardeftab720\partightenfactor0

\f0\fs22 \cf0 	\cf2 //##################### COVERAGE TEST MAIN ####################\cf0 \
\
	\cf2 /*\cf0 \
\cf2 	BSP_PB_Init(BUTTON_KEY,BUTTON_MODE_GPIO);\cf0 \
\cf2 	WaitUserButton();\cf0 \
\cf2 	char m[17];\cf0 \
\cf2 	\ul int\ulnone  cycle = 5;\cf0 \
\cf2 	while(cycle--)\cf0 \
\cf2 	\{\cf0 \
\cf2 		for(\ul int\ulnone  \ul sc\ulnone =0;\ul sc\ulnone <10;\ul sc\ulnone ++)\cf0 \
\cf2 		\{\cf0 \
\cf2 			\ul sprintf\ulnone (m, "\{\\"c\\":%d,\\"m\\":%d\}", (5-cycle), \ul sc\ulnone );\cf0 \
\cf2 			PRINTF(m);\cf0 \
\cf2 			PRINTF("\\r\\n");\cf0 \
\cf2 			Send(m);\cf0 \
\cf2 			HAL_Delay(3000);\cf0 \
\cf2 		\}\cf0 \
\cf2 		if(cycle>0)\cf0 \
\cf2 			HAL_Delay(10000);\cf0 \
\cf2 		//PRINTF("\ul Ciclo\ulnone  %d\\r\\n", (5-cycle));\cf0 \
\cf2 	\}\cf0 \
\cf2 	LED_On(LED_BLUE);\cf0 \
\cf2 	*/}