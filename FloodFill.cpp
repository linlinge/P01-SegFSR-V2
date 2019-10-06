#include "FloodFill.h"

void FloodFill::floodFillUtil(cv::Mat& screen, int x, int y, Vertices& ant) 
{ 
    // Base cases	
    if ( x < 0 || x >= screen.rows || y < 0 || y >= screen.cols) 
        return; 
    if (screen.at<uchar>(x,y) != 0) 
        return; 
  
    // Replace the color at (x, y) 
    screen.at<uchar>(x,y) = 100; 
	Vertex* vtmp=active_vertices_.Remove(x,y);
	ant.Insert(vtmp);
  
    // Recur for north, east, south and west 
    floodFillUtil(screen, x+1, y,ant); 
    floodFillUtil(screen, x-1, y,ant); 
    floodFillUtil(screen, x, y+1,ant); 
    floodFillUtil(screen, x, y-1,ant); 
}

FloodFill::FloodFill(cv::Mat& img) 
{ 
	for(int i=0;i<img.rows;i++){
		//#pragma omp parallel shared(active_vertices_)
		for(int j=0;j<img.cols;j++){
			if(img.at<uchar>(i,j)==0)
				active_vertices_.Insert(i,j);		
		}
	}

		
	// Find all result
	while(active_vertices_.number_!=0){
			Vertices ant;
			int x=active_vertices_.head_->next->i_;
			int y=active_vertices_.head_->next->j_;
			floodFillUtil(img,x,y ,ant);
			result_.push_back(ant);
	}

	// collate result
	sort(result_.begin(),result_.end(),[](Vertices& e1, Vertices& e2){return e1.number_>e2.number_;});	
} 