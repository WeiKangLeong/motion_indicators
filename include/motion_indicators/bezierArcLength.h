namespace BezierArcLength {

  //http://steve.hollasch.net/cgindex/curves/cbezarclen.html
  #define sqr(x) (x * x)
  struct point {
	  double x, y;
  };


  /************************ split a cubic bezier in two ***********************/
    
  static void bezsplit(
      point    *V,                          /* Control pts      */
      point    *Left,                       /* RETURN left half ctl pts */
      point    *Right)                     /* RETURN right half ctl pts  */
  {
      int   i, j;                               /* Index variables  */
      point   Vtemp[4][4];                      /* Triangle Matrix */
      
      /* Copy control points  */
      
      for (j =0; j <= 3; j++) 
	Vtemp[0][j] = V[j];
      

      /* Triangle computation */
      for (i = 1; i <= 3; i++) {  
      for (j =0 ; j <= 3 - i; j++) {
	  Vtemp[i][j].x =
	      0.5 * Vtemp[i-1][j].x + 0.5 * Vtemp[i-1][j+1].x;
	  Vtemp[i][j].y =
	      0.5 * Vtemp[i-1][j].y + 0.5 * Vtemp[i-1][j+1].y;
	    
      }                                       /* end for i */
      }                                       /* end for j */
      
      for (j = 0; j <= 3; j++) 
	  Left[j]  = Vtemp[j][0];
	
      for (j = 0; j <= 3; j++) 
	  Right[j] = Vtemp[3-j][j]; 
  }                                           /* end splitbez */

  /********************** add polyline length if close enuf *******************/
  double V2DistanceBetween2Points(point *a, point *b){
    return sqrt(pow(a->x-b->x,2)+pow(a->y-b->y,2));
  }
  
  static void addifclose( 
			point  *V,
			double  *length,
			double error)

  {
    point left[4], right[4];                  /* bez poly splits */
    
    double len = 0.0;                         /* arc length */
    double chord;                             /* chord length */

    
    int index;                                /* misc counter */
    
    for (index = 0; index <= 2; index++)
      len = len + V2DistanceBetween2Points(&V[index],&V[index+1]);

    chord = V2DistanceBetween2Points(&V[0],&V[3]);

    if((len-chord) > error)
    {
      bezsplit(V,left,right);                 /* split in two */
      addifclose(left, length, error);        /* try left side */
      addifclose(right, length, error);       /* try right side */
      return;
    }
    
    *length = *length + len;

    return;
    
    
  }                                                     /* end addifclose */

  /************************* arc length of a 2d bezier ************************/

  double arclen(vector<double> A, vector<double> B, double error)
  {
    
    double length;                                    /* length of curve */
    point V[4];
    for(int i=0; i<4; i++) {
      V[i].x = A[i];
      V[i].y = B[i];
    }
    addifclose(V, &length, error);                    /* kick off recursion */
    
    return(length);                                   /* that's it! */
    
  } 
  /* end arclen */
}