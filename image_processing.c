#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

int top_offset(uint8_t frame[160][120][3]) {		//function for cropping the image we have for image processing. Pre-processing function. Finds out how many rows from the top of the image are black
													//and thus we have no interest in. This makes our further image processing both easier and more accurate
	int i;	//iterator								//frame is stored as a matrix with [x][y][z], x = rows, starting from 0 = bottom row, y = columns with 0 = leftmost column and z = 1 pixel, stored as a 3 byte value
	int j;	//iterator
	int temp = 0;	//for calculating the "brightness" of a row of pixels
	int rows_black = 0;		//for holding the number of rows from the top that are "black" and not worth our time looking at

	for(i = 159; i > -1; i--) {		//iterate through all rows in the image

		for(j = 0; j < 120; j++) {		//iterate through all columns in each row

			temp += frame[i][j][0];		//add all individual pixel values, 0 = R, 1 = G, 2 = B. Since these are grayscale images, a higher total number is a brighter number, and the
			temp += frame[i][j][1];		//proportions of each individual color should always be the same. Thus we just need to add them up to find out how "bright" a pixel is.
			temp += frame[i][j][2];


		}

		if(temp > 6000) {		//if we've found a row thats bright enough to warrant attention, we've found the top of the part of the image we're interested in. Return how many rows from the top that is

			return rows_black;

		}

		else {					//else, we aren't interested in this row, go to the next one

			temp = 0;
			rows_black++;

		}

	}

	return -1;			//our image isn't bright enough to do image processing on. Bad image

}

int bottom_offset(uint8_t frame[160][120][3]) {			//same as top offset, but this time we're finding out how many rows from the bottom of the image are black, and thus not worth our time.

	int i;	//iterator
	int j;	//iterator

	int rows = 0;		//number of rows we've counted that are too dark to warrant attention
	int temp = 0;		//value for storing the total brightness of a row

	for(i = 0; i < 160; i++) {		//iterate through all the rows in the image/frame

		for(j = 0; j < 120; j++){		//iterate through all the columns in the image/frame

			temp += frame[i][j][0];		//add up the RGB value of the pixel. since black and white, the proportions don't matter, only the total value is relevant, telling us how bright a pixel is.
			temp += frame[i][j][1];
			temp += frame[i][j][2];

		}

		if(temp > 6000) {		//row is bright enough to warrant attention. We've found the bottom of the part of the image we're concerned with. Return number of rows from the bottom we aren't concerned with

			return rows;

		}

		else {		//row isn't bright enough for us to be concerned with it. Add number of rows from bottom and iterate through the next row.

			rows++;
			temp = 0;

		}

	}

	return -1;		//image must be completely black or not bright enough for us to do image processing on.

}

int right_offset(uint8_t frame[160][120][3]) {		//same as previous offset functions, but this time finding out how many COLUMNS from the right are too dark for us to be concerned with.

	int i;		//iterator
	int j;		//iterator

	int rows = 0;	//rows that are too dark
	int temp = 0;	//variable for holding the value of the pixel. Pixels are RGB encoded, but since image is black and white, only the total value matters, not the individual proportions of each color.

	for(i = 0; i < 120; i++) {		//iterate through all columns

		for(j = 0; j < 160; j++) {		//iterate through all rows in each column

			temp += frame[j][i][0];
			temp += frame[j][i][1];
			temp += frame[j][i][2];

		}

		if(temp > 6000) {		//column is bright enough to warrant our attention. return the number of columns from the right that the part of the image we're concerned with is

			return rows;

		}

		else{		//column isnt bright enough to warrant attention. Add to number of columns from the right our image is.

			rows++;
			temp = 0;

		}

	}

	return -1;

}

int left_offset(uint8_t frame[160][120][3]) {		//same as other offset functions, but this time finding our how many columns from the left our concerned image is

	int i;	//iterator
	int j;	//iterator

	int rows = 0;	//number of rows from the left are black/not bright enough
	int temp = 0;	//variable for holding the value of the pixel. Pixels are RGB encoded, but since image is black and white, only the total value matters, not the individual proportions of each color.

	for(i = 119; i > -1; i--){		//iterate through all columns

		for(j = 0; j < 160; j++) {		//iterate through all rows in each column

			temp += frame[j][i][0];
			temp += frame[j][i][1];
			temp += frame[j][i][2];

		}

		if(temp > 6000) {		//column is bright enough to warrant our attention. return the number of columns from the right that the part of the image we're concerned with is

			return rows;

		}

		else {		//column isnt bright enough to warrant attention. Add to number of columns from the right our image is.

			rows++;
			temp = 0;

		}

	}

	return -1;

}

//********************************************************************************************* may have named left_offset and right_offset backwards, i.e. left is right and right is left
//********************************************************************************************* but at this point im too afraid it may break something if i fix it so, sorry! May get around to it later though

uint8_t* process_new_frame(uint8_t*** frame) {		//function for processing a frame matrix with unknown dimensions. Not currently in use, and hasn't been tested/debugged, but I may implement it later
													//this function should only be called when passing a frame that has been pre-processed, i.e. you've gotten the offsets and cropped the image to what is relevant
	int i;		//iterator
	int j;		//iterator
	int temp = 0;		//variable for holding the value of the pixel. Pixels are RGB encoded, but since image is black and white, only the total value matters, not the individual proportions of each color.
	uint8_t* skew = (uint8_t*) calloc(4, sizeof(uint8_t));		//allocate memory for the skew, so we can return it later. C is evil sometimes.

	for(i = (sizeof(frame) / 2); i < sizeof(frame); i++) {	//iterate through our rows 		//iterate through the top half of the image, so we can get the brightness of the top half

		for(j = 0; j < sizeof(frame[i]); j++) {	//columns

			temp += frame[i][j][0];
			temp += frame[i][j][1];			//adding up brightness of each pixel in all our top half rows
			temp += frame[i][j][2];

		}

	}

	skew[0] = temp;		//assign skew [0] to be how bright the top half of our image is
	temp = 0;			//reset temp so we can get brightness of bottom half of image


	for(i = 0; i < (sizeof(frame) / 2); i++) {		//iterate through our rows 		//iterate through the bottom half of the image, so we can get the brightness of the bottom half

		for(j = 0; j < sizeof(frame[i]); j++) {		//iterate through our columns

			temp += frame[i][j][0];
			temp += frame[i][j][1];		//adding up brightness of each pixel in all our bottom half rows
			temp += frame[i][j][2];

		}

	}

	skew[1] = temp;		//assign skew[1] to be how bright the bottom half of our image is
	temp = 0;			//reset temp so we can get brightness of our left/right half

	for(i = 0; i < sizeof(frame); i++) {		//iterate through all our rows

		for(j = 0; j < (sizeof(frame[i]) / 2); j++) {		//iterate through our columns		//iterate through the left half of our image, so we can get the brightness of our left half

			temp += frame[i][j][0];
			temp += frame[i][j][1];		//adding up brightness of each pixel in our left half of columns
			temp += frame[i][j][2];

		}

	}

	skew[2] = temp;		//assign skew[2] to be how bright the left half of our image is
	temp = 0;			//reset temp so we can get brightness of our right half

	for(i = 0; i < sizeof(frame); i++) {		//iterate through all our rows

		for(j = (sizeof(frame) / 2); j < sizeof(frame); j++) {		//iterate through our columns		//iterate through the right half of our image, so we can get the brightness of our right half


			temp += frame[i][j][0];
			temp += frame[i][j][1];		//adding up brightness of each pixel in our right half of columns
			temp += frame[i][j][2];

		}

	}

	skew[3] = temp;		//assign skew[3] to be how bright the right half of our image is
	temp = 0;			//reset temp. Don't really need to I guess, but i like consistency so

	return skew;		//return out skew array

	//*****************************Not iterating through our frame matrix properly when calculating right and left half image brightness. Need to fix that if we're ever going to use this function. *******************

}



void process_frame(uint8_t frame[160][120][3], uint32_t *skew, int crops[4]) {    //an array for storing how "skewed" our image is. [0] is up, [1] down, [2] left, and [3] right.


	//*************This function is essentially the same as the above, but it modifies a preexisting skew array passed by reference to modify, and the frame to be processed has fixed, pre-known dimensions.
	//*************Additionally, it requires you to pass an array with the number of rows/columns you aren't concerned with, figured out with the x_offset(frame) function.
	//*************Will probably modify this function to call those offset functions itself, instead of requiring to pass the results of those functions as a separate argument.

     uint8_t i;
     uint8_t j;
     int half_way_down = (crops[1] + (160 - crops[1] - crops[0]) / 2);			//for finding where the middle of our image is from the top, including the cropped out parts.
     int half_way_right = (120 - crops[3] - ((120 - crops[2] - crops[3]) / 2));		//for finding out where the middle of our image is from the left, including the cropped out parts

     //^^^^^^^Both important for finding out where our half images are, so we can accurately find our how bright our actual halves are. function won't work without these.


     uint32_t temp = 0;     //used for each loop to calculate its skew. Then copied into array.

     for(i = half_way_down; i < (160-crops[0]) ; i++){       //for processing how "up" our image is skewed. Iterate through all our rows from middle of our area of interest to the top of our area of interest


         for(j = crops[2]; j < (120 - crops[3]); j++){			//iterate through all our columns in each row that we're iterating with. only including columns that we've left in our area of interest

        	 int temp2 = 0;

             temp2 = (temp2 + frame[i][j][0]) * (i - half_way_down + 1);
             temp2 = (temp2 + frame[i][j][1]) * (i - half_way_down + 1);			//add up the RGB values of the pixels. add weight to pixels that are further from the center of the image.
             temp2 = (temp2 + frame[i][j][2]) * (i - half_way_down + 1);

             if(temp2 < 10) {

            	 	 //if its an almost entirely black pixel, ignore it.

			  }

			  else if(temp2 < 100) {

				 temp = temp2 * 10;			//if its a very dim pixel, give it small weight

			  }

			  else{

				 temp = temp2 * 100;		//if its a brighter pixel, give it heavy weight

			  }

         }

     }

     skew[0] = temp;		//assign skew[0] to be how up our image is

     temp = 0;		//reset temp so we can calculate how down our image is next


     for(i = crops[1]; i < half_way_down; i++){     //for processing how "down" our image is skewed. Iterate through all our rows from bottom of area of interest to the middle of our area of interest

         for(j = crops[2]; j < (120 - crops[3]); j++){		//iterate through all columns in each row within our area of interest

        	 int temp2 = 0;

             temp2 = (temp2 + frame[i][j][0]) * (half_way_down - i + 1);
             temp2 = (temp2 + frame[i][j][1]) * (half_way_down - i + 1);		//add up the RGB values of the pixels. add weight to pixels that are further from the center of the image.
             temp2 = (temp2 + frame[i][j][2]) * (half_way_down - i + 1);

             if(temp2 < 10) {

            	 //if its an almost entirely black pixel, ignore it.

			  }

			  else if(temp2 < 100) {

				 temp = temp2 * 10;		//if its a very dim pixel, give it small weight

			  }

			  else{

				 temp = temp2 * 100;		//if its a brighter pixel, give it heavy weight

			  }


         }

     }

     skew[1] = temp;		//assign skew[1] as how bright the bottom portion of our image is

     temp = 0;				//reset temp so we can calculate our left our image is

     for(j = crops[2]; j < half_way_right; j ++) {     //for processing how "left" our image is. Iterate through all columns from left of area of interest to the middle area of interest

         for(i = crops[1]; i < (160 - crops[0]); i++) {		//iterate through all our rows within our area of interest.

        	 int temp2 = 0;

             temp2 = (temp2 + frame[i][j][0]) * (half_way_right - j + 1);
             temp2 = (temp2 + frame[i][j][1]) * (half_way_right - j + 1);		//add up the RGB values of the pixels. add weight to pixels that are further from the center of the image.
             temp2 = (temp2 + frame[i][j][2]) * (half_way_right - j + 1);

             if(temp2 < 10) {

            	 //if its an almost entirely black pixel, ignore it.

			  }

			  else if(temp2 < 100) {

				 temp = temp2 * 10;		//if its a very dim pixel, give it small weight

			  }

			  else{

				 temp = temp2 * 100;		//if its a brighter pixel, give it heavy weight

			  }

         }

     }

     skew[2] = temp;	//assign skew[2] to be how left our image is

     temp = 0;		//reset temp so we can calculate how right our image is

     for(j = half_way_right; j < (120 - crops[3]); j++) {		//for processing how right our image is. Iterate through all columns from the right of our area of interest to the center of our area of interest

         for(i = crops[1]; i < (160 - crops[0]); i++) {		//iterate through all rows within our area of interest

        	 int temp2 = 0;

             temp2 = (temp2 + frame[i][j][0]) * (j - half_way_right + 1);
             temp2 = (temp2 + frame[i][j][1]) * (j - half_way_right + 1);		//add up the RGB values of the pixels. add weight to pixels that are further from the center of the image.
             temp2 = (temp2 + frame[i][j][2]) * (j - half_way_right + 1);

             if(temp2 < 10) {

            	 //if its an almost entirely black pixel, ignore it.

             }

             else if(temp2 < 100) {

            	 temp = temp2 * 10;		//if its a very dim pixel, give it small weight

             }

             else{

            	 temp = temp2 * 100;		//if its a brighter pixel, give it heavy weight

             }

         }

     }

     skew[3] = temp;		//assign skew[3] to be how "right" our image is

     return;


     //***********Will need to adjust how we do the weighting on each pixel.

 };


void movements(uint32_t skew[4], int *motor_commands) {			//function for converting our skew to more managable numbers, easier to send to functions to that actually send commands to the physical motors
																//to adjust our collimation

     float temp = 0;     //temporary variable for doing calculations


     if(skew[0] > skew[1]) {    //we need to adjust our collimation upwards, from the image's perspective

         temp = skew[0] - skew[1];
         temp = temp /  100000;


         motor_commands[0] = temp;


     }

     else{  //we need to adjust our collimation downwards, from the image's perspective

         temp = skew [1] - skew[0];
         temp = temp / 100000;


         motor_commands[1] = temp;


     }

     temp = 0;

     if(skew[2] > skew[3]) {        //we need to adjust our collimation rightwards, from the image's perspective

         temp = skew[2] - skew [3];         //find our how skewed exactly we are to the left;
         temp = temp / 100000;             //find our how skewed we are in comparison to the maximum amount of skewed we can be. Max skewed would assume right completely black
                                            //and left completely white, giving difference of 2448000. This would of course mean our image is off center instead of our collimation image being actually skewed in relation to
                                            //itself, but for now we'll assume our image is completely centered on our camera, and any difference is due to our collimation being skewed, instead of artifacts in the image
         	 	 	 	 	 	 	 	 	 //get a percentage.

         motor_commands[2] = temp;

     }

     else {     //we need to adjust our collimation leftwards, from the image's perspective


         temp = skew[3] - skew[2];
         temp = temp / 100000;

         motor_commands[3] = temp;

     }

     return;

     //*********Will need to adjust the specific numbers here as well, but the algorithm itself isn't bad

 }



int main() {

	int l;

	for(l = 0; l < 13; l++) {

		FILE *fp;
		char* current_file;

		switch(l) {

			case 0:
				fp = fopen("2_4.bmp", "rb");
				current_file = "2_4.bmp";
				break;

			case 1:
				fp = fopen("2_3.bmp", "rb");
				current_file = "2_3.bmp";
				break;

			case 2:
				fp = fopen("2_2.bmp", "rb");
				current_file = "2_2.bmp";
				break;

			case 3:
				fp = fopen("2_1.bmp", "rb");
				current_file = "2_1.bmp";
				break;

			case 4:

				fp = fopen("5.bmp", "rb");
				current_file = "5.bmp";
				break;

			case 5:

				fp = fopen("4.bmp", "rb");
				current_file = "4.bmp";
				break;

			case 6:

				fp = fopen("3.bmp", "rb");
				current_file = "3.bmp";
				break;

			case 7:

				fp = fopen("2.bmp", "rb");
				current_file = "2.bmp";
				break;

			case 8:

				fp = fopen("1.bmp", "rb");
				current_file = "1.bmp";
				break;

			case 9:

				fp = fopen("3_4.bmp", "rb");
				current_file = "3_4.bmp";
				break;

			case 10:

				fp = fopen("3_3.bmp", "rb");
				current_file = "3_3.bmp";
				break;

			case 11:

				fopen("3_2.bmp", "rb");
				current_file = "3_2.bmp";
				break;

			case 12:

				fopen("3_1.bmp", "rb");
				current_file = "3_1.bmp";
				break;

		}

		int i;
		int k;
		int j;
		fseek(fp, 0, SEEK_SET);			//open up one of our BMP (bitmap) images and set our file pointer to the beginning of it.

		unsigned char info[148];		//for storing our bitmap header info

		fread(info, sizeof(unsigned char), 148, fp);		//read our bitmap header info and put it into our info array

		//int width = *(int*)&info[18];	//width of our image in pixels
		//int height = *(int*)&info[22];	//height of our image in pixels
		int bitmap_offset = *(int*)&info[10];	//offset from the beginning of the bitmap file to the beginning of the actual pixel array.
		//int bitmap_size = *(int*)&info[34];		//size in byte of the bitmap array
		//int header_size = *(int*)&info[14];		//size of our bitmap file's header
		fseek(fp, bitmap_offset, SEEK_SET);		//point file pointer to the beginning of the bitmap array;

		//printf("Bitmap size: %d\nHeader size: %d\n", bitmap_size, header_size);

		//printf("Width is %d \nHeight is %d\n", width, height);

		uint8_t data[57600]; // allocate 3 bytes per pixel. Array for reading the entire bitmap array. 120 x 160 pixels, 19200 total pixels, 3 bytes per pixel, 57600 total bytes in bitmap array
		fread(data, 1, 57600, fp); // read the entire bitmap array into our data array
		fclose(fp);

		uint8_t frame[160][120][3];		//create frame matrix for easier storing of the pixels.


		j = 0;
		k = 0;

		for(i = 0; i < sizeof(data); i+=3){

			frame[k][j][0] = data[i];
			frame[k][j][1] = data[i + 1];			//store each pixel's data into our matrix for easier manipulation
			frame[k][j][2] = data[i + 2];

			if(j == 119) {

				k++;
				j = 0;

			}

			else {

				j++;

			}

		}

		int crops[4] = {0, 0, 0, 0};

		crops[0] = top_offset(frame);
		crops[1] = bottom_offset(frame);
		crops[2] = left_offset(frame);
		crops[3] = right_offset(frame);				//get our dimensions for cropping our image. Basically this just finds our how many rows/columns from the top/bottom and left/right are black, and thus not
													//relevant when doing our image processing. Doing this pre-processing makes our image processing faster and more accurate.
		uint32_t skew[4] = {0, 0, 0, 0};
		int motor_commands[4] = {0, 0, 0, 0};

		process_frame(frame, skew, crops);			//process our frame and put into the skew array how bright each top/bottom and left/right half of our image is.
		movements(skew, motor_commands);			//convert our skew into numbers we can more easily pass to functions that move the physical motors.

		//int top_skew_frame = (crops[1] + (160 - crops[1] - crops[0]) / 2);
		//int bottom_skew_frame = (160 - crops[0] - ((160 - crops[1] - crops[0]) / 2));
		//int left_skew_frame = (120 - crops[3] - ((120 - crops[2] - crops[3]) / 2));
		//int right_skew_frame = (crops[2] + ((120 - crops[2] - crops[3]) / 2));

		//printf("Top skew frame %d\nBottom skew frame %d\nLeft skew frame %d\nRight skew frame %d\n", top_skew_frame, bottom_skew_frame, left_skew_frame, right_skew_frame);



		printf("%s:    ", current_file);
		printf("Up %d   Down %d   Right %d   Left %d\n", motor_commands[0], motor_commands[1], motor_commands[2], motor_commands[3]);		//lets print our motor commands!

		//printf("Up offset %d\nDown offset %d\nLeft offset %d\nRight offset %d\n", crops[0], crops[1], crops[2], crops[3]);




	}

	return 0;
}
