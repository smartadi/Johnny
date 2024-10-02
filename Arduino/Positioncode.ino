#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <math.h>



// Define the maximum number of sensors
const int MAX_SENSORS = 4;  // Adjust this value as needed also called m in places

// structure to store updated position -> easiest way to pass it through a function
struct Position
{
    float x_pos;
    float y_pos;
    float z_pos;
};

// Function to calculate the norm of a 3x1 vector
float vectorNorm(const BLA::Matrix<3, 1> &vec) {
    // Calculate the Euclidean (L2) norm
    float norm = sqrt(pow(vec(0, 0), 2) + pow(vec(1, 0), 2) + pow(vec(2, 0), 2));
    return norm;
}

// function that return the transpose of a matrix
BLA::Matrix<3, MAX_SENSORS> transposeMatrix(const BLA::Matrix<MAX_SENSORS, 3> &mat) {
    BLA::Matrix<3, MAX_SENSORS> transposedMat;  // Create the transposed matrix (3 x m)

    // Transpose operation: Swap rows and columns
    for (int i = 0; i < MAX_SENSORS; i++) {
        for (int j = 0; j < 3; j++) {
            transposedMat(j, i) = mat(i, j);
        }
    }

    return transposedMat;
}

 /**
    Estimate the object's position using gradient descent.
    
    Parameters:
    - sensor_positions:   array of shape (m, 3)
    - measured_distances:  array of shape (m,)
    - initial_guess:  array of shape (3,)
    - max_iterations: int, maximum number of iterations (inside function for now can be made a variable)
    - tolerance: float, convergence threshold            (inside function for now can be made a variable)
    
    Returns:
    - estimated_position: array of shape (3,), estimated object position
  **/
Position findPos(BLA::Matrix<MAX_SENSORS,3> &SEN_POS, 
              BLA::Matrix<MAX_SENSORS> &DIST_M, 
              BLA::Matrix<3> &INIT_GUESS) {
   
  int iterations = 100;
  float tolerance = .0001;
    
  BLA::Matrix<MAX_SENSORS,3> A;
  BLA::Matrix<MAX_SENSORS> DIST_EST;

  BLA::Matrix<3> x = INIT_GUESS; // current position estimate
  BLA::Matrix<3> x_new;
  BLA::Matrix<3> s_i; // Sensor position
  float r_i; // measured distance
  float p_i; // = || x - s_i ||  current estimate distance
  float d_i; // = r_i - p_i residual betweem estimated and measured distance
  float tol_check;
  BLA::Matrix<3> a_i; // direction vector = (x-s_i)/ p_i
  
  BLA::Matrix<3, MAX_SENSORS> AT;
  BLA::Matrix<3, 3> ATA;
  BLA::Matrix<3, 3> ATA_INV;
  BLA::Matrix<3, MAX_SENSORS> A_PLUS;
  BLA::Matrix<3> estimated_pos_delta; 

  
  // iteratively solve until tolerance is reached 
  for (int i = 0; i < iterations; i++) {
    Serial.println(i);
    //Build A and d matrix one sensor at a time
    for (int i = 0; i < MAX_SENSORS; i++) {
      
      // extract sensor position and dsitance measurements
      s_i(0) = SEN_POS(i, 0); // 1x3 matrix 
      s_i(1) = SEN_POS(i, 1); // 1x3 matrix 
      s_i(2) = SEN_POS(i, 2); // 1x3 matrix 
      
      r_i = DIST_M(i); // scalar
      

      // compute estimated distance between position and sensor
      p_i = vectorNorm(x-s_i); 
      
      // compute residual distance betweenn measurment and estimate
      d_i = r_i - p_i;
        
      // compute unit vector of position to sensor 
      a_i = (x - s_i)/p_i;
      // build A matrix 
      A(i, 0) = a_i(0);
      A(i, 1) = a_i(1);
      A(i, 2) = a_i(2);
      // build distance matrix
      DIST_EST(i) = d_i;
    }
    //find pseudoinverse 
      
    // compute transpose of A
    AT = transposeMatrix(A);
    ATA = AT * A;
    ATA_INV = Inverse(ATA);

    A_PLUS = ATA_INV * AT;
      
    // solve for delta estimated position
    estimated_pos_delta = A_PLUS * DIST_EST; 
      
    // solve for new estimate
    x_new = x + estimated_pos_delta;

    Serial.print("x_new: ");
    Serial.println(x_new);

    // check tolerance
    tol_check = vectorNorm(x_new - x);

    if (tol_check < tolerance) {
      return {x(0), x(1), x(2)};
      
    }
    // update position estimate
    x = x_new;
  }
  return {x(0), x(1), x(2)};
}

void setup()
{
  Serial.begin(9600);

  using namespace BLA;

  BLA::Matrix<MAX_SENSORS,3> SEN_POS;
  BLA::Matrix<MAX_SENSORS> DIST_M;
  BLA::Matrix<3> INIT_GUESS; 
   
  // update for real positions, also update number of sensors at top if that changes
  SEN_POS = {0,  0, 15,  //Sensor 1 Position
             20, 20, 15, //Sensor 2 Position
             20,  0, 20, //Sensor 3 Position
             10, 10, 15}; //Sensor 4 Position

  DIST_M = {16.995, 20.251, 20.9258, 12.4559}; // Will gather from tag 
                                               // this is a test case

  // same bet is to just leave it at 0,0,0
  INIT_GUESS = {0,0,0};   

  // generate position estimate 
  Position POS = findPos(SEN_POS, 
                         DIST_M, 
                         INIT_GUESS); 

  float POSx = POS.x_pos;
  float POSy = POS.y_pos;
  float POSz = POS.z_pos;

  Serial.println("Position should be 10, 7, 3");
  Serial.println(POSx, 5);
  Serial.println(POSy, 5);
  Serial.println(POSz, 5);

}


void loop() {
  
}
