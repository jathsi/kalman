#include <ros/ros.h>
#include "kalman/Roshdl.h"
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <fstream>  
#include <string>
#include "kalman/kalman.h"
#include <cmath>
 


void Roshdl::split(unsigned int p_t1, unsigned int p_t2) {
 
  double slope,angle_,radius_ ;
  int x1 = xs[p_t1];
  int y1 = ys[p_t1];
  int x2 = xs[p_t2];
  int y2 = ys[p_t2];
  double dist,dist_max=0.5,part1=1,part2=1;
  int i_max=0;

   for (std::size_t i = p_t1+1; i < p_t2; ++i)
  {
    //dist = distToPoint(i,angle_,radius_);

    //dist = double((abs(fabs(((y2-y1)*xs[i])))-fabs(((x2-x1)*ys[i]))+fabs((x2*y1))-fabs((x1*y2)))/sqrt(fabs((pow((y2-y1),2)))+fabs(pow((x2-x1),2))));

    part1=fabs(abs(((y2-y1)*xs[i])-((x2-x1)*ys[i])+(x2*y1)-(x1*y2)));
    part2=fabs(sqrt(fabs((pow((y2-y1),2)))+fabs(pow((x2-x1),2))));
    dist = fabs(part1/part2);

    if (dist > dist_max)
    {
      dist_max = dist;
      i_max = i;
     // ROS_DEBUG("%d,%f,%f,%f",i,part1,part2,dist);
     
    }
 

  }
  if (dist_max>0.5&&!isinf(dist_max)){
    ROS_DEBUG("splitpoint %d %f",i_max,dist_max);

    split(p_t1,i_max-1);
    split(i_max,p_t2);

  }
  else{
          //ROS_DEBUG("before the check segmentation %d %d %f",p_t1,p_t2,dist_max);

    if(!isinf(dist_max)){
  seg1.push_back(p_t1);
  seg1.push_back(p_t2);

  

}
  }

  
}

Roshdl::Roshdl(ros::NodeHandle& nh, ros::NodeHandle& nh_loc_):
  nh_(nh),
  nh_loc_(nh_loc_)
{
  scan_sub_ = nh_.subscribe("node1/base_scan", 1, &Roshdl::ScanarrivalFn, this);
  marker_publ_ = nh_.advertise<visualization_msgs::Marker>("line_markers", 1);

}
double Roshdl::distToPoint(unsigned int index, double angle_, double radius_)
{
  double p_rad = sqrt(pow(xs[index], 2) + pow(ys[index], 2));
  double p_ang = atan2(ys[index],xs[index]);
  return fabs(p_rad * cos(p_ang - angle_) - radius_);
}
double Roshdl::pi_to_pi(double angle)
{
  angle = fmod(angle, 2 * M_PI);
  if (angle >= M_PI)
    angle -= 2 * M_PI;
  return angle;
}
void Roshdl::ScanarrivalFn(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{


  std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(), scan_msg->ranges.end());
  std::vector<unsigned int> indices;
  unsigned int i = 0;
  bearings.clear();
    cos_bearings.clear();
    cos_bearings.clear();
    sin_bearings.clear();
    indices.clear();
  for (double b = scan_msg->angle_min; b <= scan_msg->angle_max; b += scan_msg->angle_increment)
  {


    bearings.push_back(b);
    cos_bearings.push_back(cos(b));
    sin_bearings.push_back(sin(b));
    indices.push_back(i);
    ++i;
    //ROS_DEBUG("%d %f %f %f",i,b,cos(b),sin(b));
  }
 
   int n = 2; // Number of states
    int m = 1;
 int i1 = 0;
  ranges = scan_ranges_doubles;
  xs.clear();
  ys.clear();
  for (std::vector<unsigned int>::const_iterator cit = indices.begin(); 
       cit != indices.end(); ++cit)
  {
      xs.push_back(cos_bearings[*cit] * ranges[*cit]);
      ys.push_back(sin_bearings[*cit] * ranges[*cit]);
     // ROS_DEBUG("%f %f",ranges[*cit],cos_bearings[*cit]);
      Ranges[*cit]=ranges[*cit];
      

  }


 
  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n);
  Eigen::MatrixXd P0(n, n);
  Eigen::MatrixXd x0(n, m); // Estimate error covariance

  // Discrete LTI projectile motion, measuring position only
  A << 1, 0.00436717644, 0, 1;
  C << 1, 0;

  // Reasonable covariance matrices
  Q << .01, .01, .01, .01;
  R << .0009;
  P << 4, 0, 4, 0;
  
 // std::cout << "A: \n" << A << std::endl;
  //std::cout << "C: \n" << C << std::endl;
  //std::cout << "Q: \n" << Q << std::endl;
  //std::cout << "R: \n" << R << std::endl;
  //std::cout << "P: \n" << P << std::endl;
  geometry_msgs::Point pt1,pt2line;
  visualization_msgs::Marker points,line_list;
  points.header.frame_id = line_list.header.frame_id="test_frame";
  points.header.stamp=line_list.header.stamp= ros::Time::now();
  points.ns=line_list.ns="points_and_lines";
  points.action=line_list.action=visualization_msgs::Marker::ADD;
  points.pose.orientation.w=line_list.pose.orientation.w=1.0;
  points.id=0;
  line_list.id=1;
  points.type=visualization_msgs::Marker::POINTS;
  line_list.type=visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x=0.5;
  line_list.color.r=1.0;
  line_list.color.a=1;
  points.color.g=1.0f;
  points.color.a=1;
  points.scale.x=0.2;
  points.scale.y=0.2;

  // Construct the filter
  KalmanFilter kf(A, C, Q, R, P);

  Eigen::MatrixXd y(m,m);
  
  //double Ranges[720]={3.5040934085845947, 3.503842830657959, 3.497749090194702, 3.4901247024536133, 3.509298324584961, 3.5055668354034424, 3.5024545192718506, 3.5218098163604736, 3.5233895778656006, 3.50457501411438, 3.515747308731079, 3.509408473968506, 3.5220518112182617, 3.5022010803222656, 3.51765513420105, 3.5206830501556396, 3.5221047401428223, 3.537569284439087, 3.54447340965271, 3.5369272232055664, 3.5473928451538086, 3.5301284790039062, 3.536062002182007, 3.530336380004883, 3.517821788787842, 3.5479226112365723, 3.5189692974090576, 3.5333147048950195, 3.536616086959839, 3.51690936088562, 3.546506881713867, 3.540005922317505, 3.5468263626098633, 3.551593780517578, 3.5333189964294434, 3.5476956367492676, 3.5544850826263428, 3.560598611831665, 3.564129114151001, 3.572896718978882, 3.57590651512146, 3.5661370754241943, 3.576167583465576, 3.5714333057403564, 3.581220865249634, 3.5953853130340576, 3.5782570838928223, 3.592802047729492, 3.592735528945923, 3.6043930053710938, 3.611611843109131, 3.605949640274048, 3.6079208850860596, 3.6140570640563965, 3.62150239944458, 3.6035923957824707, 3.62827205657959, 3.6331911087036133, 3.617422342300415, 3.636691093444824, 3.631617307662964, 3.6492748260498047, 3.6406731605529785, 3.63885235786438, 3.645888090133667, 3.657181978225708, 3.6741058826446533, 3.679565906524658, 3.6783759593963623, 3.690627098083496, 3.6967031955718994, 3.6816563606262207, 3.6982619762420654, 3.689868211746216, 3.7021727561950684, 3.682090997695923, 3.7054967880249023, 3.7172138690948486, 3.7304935455322266, 3.7170305252075195, 3.7326927185058594, 3.7323966026306152, 3.744089365005493, 3.743600845336914, 3.753425359725952, 3.755054235458374, 3.7739200592041016, 3.7704145908355713, 3.774174213409424, 3.799941301345825, 3.801546812057495, 3.8024094104766846, 3.8200783729553223, 3.8240272998809814, 3.815006971359253, 3.835908889770508, 3.851330518722534, 3.8586907386779785, 3.84950852394104, 3.8667218685150146, 3.8629772663116455, 3.8766934871673584, 3.8877642154693604, 3.9088382720947266, 3.900538444519043, 3.9139225482940674, 3.9092373847961426, 3.926934242248535, 3.9423019886016846, 3.9454848766326904, 3.9699504375457764, 3.985924005508423, 3.968188762664795, 3.9825379848480225, 4.009275913238525, 4.016845703125, 3.998717784881592, 4.024711608886719, 4.035574436187744, 4.0398783683776855, 4.072787761688232, 4.073999404907227, 4.072398662567139, 4.0711798667907715, 4.0963592529296875, 4.109280109405518, 4.1039934158325195, 4.126735210418701, 4.128833293914795, 4.135591506958008, 4.166055202484131, 4.190983295440674, 4.176619529724121, 4.199076175689697, 4.224144458770752, 4.219086170196533, 4.2327046394348145, 4.245365619659424, 4.246628761291504, 4.290388107299805, 4.292315483093262, 4.312115669250488, 4.310606002807617, 4.3368096351623535, 4.33917236328125, 4.345829963684082, 4.377563953399658, 4.38394832611084, 4.393985271453857, 4.445576190948486, 4.416876792907715, 4.414933204650879, 4.46167516708374, 4.481083393096924, 4.493995189666748, 4.486677169799805, 4.509978771209717, 4.534032821655273, 4.543749809265137, 4.561600208282471, 4.583635330200195, 4.606708526611328, 4.625672340393066, 4.648148059844971, 4.653050422668457, 4.679790019989014, 4.691413879394531, 4.690507888793945, 4.724708080291748, 4.77345609664917, 4.759065628051758, 4.776480197906494, 4.810232162475586, 4.837325572967529, 4.837263584136963, 4.88442325592041, 4.894008159637451, 4.8951334953308105, 4.918054103851318, 4.963290214538574, 4.970210075378418, 5.008591175079346, 5.01182746887207, 5.040318012237549, 5.054755687713623, 5.108839511871338, 5.120354175567627, 5.133243560791016, 5.140745162963867, 5.162424564361572, 5.186045169830322, 5.234438896179199, 5.256148338317871, 5.286172866821289, 5.300766944885254, 5.34993314743042, 5.349283218383789, 5.387414455413818, 5.398448944091797, 5.45150089263916, 5.463925838470459, 5.5173211097717285, 5.539465427398682, 5.542693138122559, 5.606103420257568, 5.611627578735352, 5.6511454582214355, 5.674943447113037, 5.713498592376709, 5.734604358673096, 5.754673480987549, 5.804853916168213, 5.854167938232422, 5.8609619140625, 5.934478282928467, 5.93696928024292, 5.983003616333008, 6.031136512756348, 6.051295757293701, 6.0999627113342285, 6.147875785827637, 6.183457851409912, 6.219549179077148, 6.250777244567871, 6.291306972503662, 6.347926139831543, 6.366839408874512, 6.425957202911377, 6.454233646392822, 6.5107574462890625, 6.542913913726807, 6.62145471572876, 6.6095290184021, 6.68934440612793, 6.714754104614258, 6.773805141448975, 6.847564697265625, 6.87161922454834, 6.837048053741455, 6.822537899017334, 6.822336196899414, 6.769773960113525, 6.794790267944336, 6.765547752380371, 6.759699821472168, 6.732940196990967, 6.719945907592773, 6.689648628234863, 6.679342746734619, 6.67094612121582, 6.647990703582764, 6.664460182189941, 6.627849578857422, 6.617956161499023, 6.590484142303467, 6.598043441772461, 6.565463542938232, 6.567074775695801, 6.522164344787598, 6.536162853240967, 6.5448994636535645, 6.488804817199707, 6.500815391540527, 6.4671430587768555, 6.458039283752441, 6.469449520111084, 6.435588836669922, 6.41602897644043, 6.411436080932617, 6.400735378265381, 6.3690595626831055, 6.383150577545166, 6.3907151222229, 6.3617048263549805, 6.340242862701416, 6.32710075378418, 6.307368278503418, 6.327662467956543, 6.308835506439209, 6.29764986038208, 6.290152549743652, 6.266629219055176, 6.261727809906006, 6.233711242675781, 6.23573637008667, 6.227930545806885, 6.2251081466674805, 6.218541622161865, 6.203773021697998, 6.186186790466309, 6.194339752197266, 6.18800687789917, 6.177974224090576, 6.1860151290893555, 6.170523166656494, 6.151113986968994, 6.149371147155762, 6.153357028961182, 6.121579170227051, 6.123112678527832, 6.106735706329346, 6.108001708984375, 6.088056564331055, 6.1024017333984375, 6.074912071228027, 6.070611476898193, 6.077483177185059, 6.067508220672607, 6.065786361694336, 6.0613274574279785, 6.0452728271484375, 6.03751802444458, 6.038980960845947, 6.018548965454102, 6.028412818908691, 6.040128231048584, 6.021772861480713, 6.005455017089844, 5.992190361022949, 6.0070295333862305, 5.992008209228516, 5.992410659790039, 1.7698966264724731, 1.7690205574035645, 1.7748866081237793, 1.7809404134750366, 1.7671889066696167, 1.7750225067138672, 1.76656174659729, 1.7763687372207642, 1.7452481985092163, 1.7692763805389404, 1.7463300228118896, 1.7766869068145752, 1.7710663080215454, 1.7645832300186157, 1.7582275867462158, 1.7741451263427734, 1.7749841213226318, 1.753159999847412, 1.768855333328247, 1.759366512298584, 1.7424497604370117, 1.7564200162887573, 1.752838134765625, 1.7608588933944702, 1.7628499269485474, 1.7401530742645264, 1.74744713306427, 1.7523078918457031, 1.7443857192993164, 1.7567654848098755, 1.7392630577087402, 1.7593761682510376, 1.7573411464691162, 1.7491363286972046, 1.7629314661026, 1.761352300643921, 1.75490403175354, 1.7641760110855103, 1.7670750617980957, 1.7557718753814697, 1.7461621761322021, 1.7668145895004272, 1.7638760805130005, 1.7570608854293823, 1.7345839738845825, 1.73561692237854, 1.7653517723083496, 1.759576678276062, 1.7528187036514282, 1.7569975852966309, 1.7422232627868652, 1.7707717418670654, 1.7501386404037476, 1.74811851978302, 1.7523434162139893, 1.754730224609375, 1.7588775157928467, 1.7652369737625122, 1.7480710744857788, 1.7653475999832153, 1.7434438467025757, 1.765022873878479, 1.752529501914978, 1.7359282970428467, 1.7882908582687378, 5.935822010040283, 5.949547290802002, 5.963839054107666, 5.955036640167236, 5.976120948791504, 5.976733207702637, 5.959146022796631, 5.960581302642822, 5.965818405151367, 5.986139297485352, 5.997229099273682, 5.9934186935424805, 5.984531402587891, 5.990903377532959, 6.00704288482666, 5.997311115264893, 5.995550632476807, 6.01252555847168, 6.017401695251465, 6.03506326675415, 6.0358076095581055, 6.024750232696533, 6.04818868637085, 6.065150260925293, 6.041072845458984, 6.071485996246338, 6.081900596618652, 6.067013263702393, 6.072169780731201, 6.0739545822143555, 6.0944719314575195, 6.097612380981445, 6.104806900024414, 6.097754001617432, 6.125818729400635, 6.141902446746826, 6.1296586990356445, 6.1583757400512695, 6.151017665863037, 6.150716304779053, 6.163236618041992, 6.175967693328857, 6.180966377258301, 6.197528839111328, 6.191166400909424, 6.211380958557129, 6.216614723205566, 6.230084419250488, 6.224534034729004, 6.255397796630859, 6.2692365646362305, 6.273481369018555, 6.27795934677124, 6.303277492523193, 6.283200263977051, 6.218878269195557, 6.153631210327148, 6.087641716003418, 6.021829128265381, 5.956845760345459, 5.878558158874512, 5.807102203369141, 5.745894432067871, 5.7173967361450195, 5.649977207183838, 5.578622817993164, 5.515936851501465, 5.479738712310791, 5.400619029998779, 5.375211715698242, 5.306675910949707, 5.267374038696289, 5.207082271575928, 5.1563849449157715, 5.117382526397705, 5.086292266845703, 5.023852348327637, 4.964268684387207, 4.939894676208496, 4.903604507446289, 4.860572814941406, 4.79413366317749, 4.763389587402344, 4.733143329620361, 4.67748498916626, 4.632579803466797, 4.623810291290283, 4.58351993560791, 4.517382621765137, 4.5116071701049805, 4.466150283813477, 4.421998977661133, 4.403871536254883, 4.370850563049316, 4.323522567749023, 4.325008392333984, 4.2797346115112305, 4.246227741241455, 4.213907241821289, 4.163663387298584, 4.147621154785156, 4.131200313568115, 4.095425605773926, 4.042143821716309, 4.046182155609131, 4.00065279006958, 3.966539144515991, 3.956509828567505, 3.922043800354004, 3.9260075092315674, 3.8842415809631348, 3.853938579559326, 3.8295958042144775, 3.7960243225097656, 3.787149667739868, 3.769512414932251, 3.7337334156036377, 3.718545436859131, 3.683558225631714, 3.66330885887146, 3.6357369422912598, 3.619917154312134, 3.609740734100342, 3.5935850143432617, 3.576181173324585, 3.5574824810028076, 3.5287139415740967, 3.5261402130126953, 3.474970817565918, 3.4695754051208496, 3.4742958545684814, 3.442103862762451, 3.4380266666412354, 3.438342571258545, 3.443537712097168, 3.456681728363037, 3.4858767986297607, 3.4961304664611816, 3.481004238128662, 3.513547897338867, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 2.7749032974243164, 2.7603185176849365, 2.7620670795440674, 2.766249418258667, 2.737083911895752, 2.7595014572143555, 2.7199954986572266, 2.732881784439087, 2.702657699584961, 2.691321611404419, 2.6982431411743164, 2.690845489501953, 2.6757264137268066, 2.643247127532959, 2.671863555908203, 2.6689062118530273, 2.636230707168579, 2.6353259086608887, 2.643099546432495, 2.629293918609619, 2.6353678703308105, 2.611436367034912, 2.5978031158447266, 2.591053009033203, 2.576202630996704, 2.5819947719573975, 2.571842908859253, 2.559805393218994, 2.5585155487060547, 2.5616583824157715, 2.543583631515503, 2.5426580905914307, 2.517033100128174, 2.5187253952026367, 2.5250027179718018, 2.499958038330078, 2.5107126235961914, 2.5095150470733643, 2.4989967346191406, 2.4885923862457275, 2.4671881198883057, 2.497540235519409, 2.476933240890503, 2.491703748703003, 2.4719207286834717, 2.45548152923584, 2.4543282985687256, 2.44425630569458, 2.433685302734375, 2.4377851486206055, 2.4346165657043457, 2.422178268432617, 2.4212472438812256, 2.430868148803711, 2.401381731033325, 2.413142204284668, 2.398775577545166, 2.402358055114746, 2.4122700691223145, 2.366691827774048, 2.388180732727051, 2.3899552822113037, 2.3745882511138916, 2.368021011352539, 2.3520216941833496, 2.3645009994506836, 2.35465145111084, 2.3607521057128906, 2.340139865875244, 2.339477300643921, 2.3453028202056885, 2.3477864265441895, 2.3279073238372803, 2.3207480907440186, 2.3181073665618896, 2.322309970855713, 2.3139450550079346, 2.3259546756744385, 2.3072543144226074, 2.297696352005005, 2.2877840995788574, 2.2946665287017822, 2.3036398887634277, 2.2980523109436035, 2.288883686065674, 2.2726328372955322, 2.2983756065368652, 2.285560369491577, 2.2842180728912354, 2.264544725418091, 2.2717905044555664, 2.2697930335998535, 2.274416923522949, 2.277157783508301, 2.265468120574951, 2.2704057693481445, 2.2653112411499023, 2.264718532562256, 2.248270273208618, 2.256849527359009, 2.2508809566497803, 2.2596473693847656, 2.2513325214385986, 2.253403425216675, 2.2349276542663574, 2.2187254428863525, 2.248464822769165, 2.2344226837158203, 2.2134342193603516, 2.2324955463409424, 2.229889154434204, 2.215155601501465, 2.2451374530792236, 2.2281246185302734, 2.2358124256134033, 2.2227373123168945, 2.2290806770324707, 2.238530158996582, 2.220038890838623, 2.219240188598633, 2.2101826667785645, 2.20005202293396, 2.205580234527588, 2.220064640045166, 2.206509828567505, 2.2126269340515137, 2.201320171356201, 2.203550100326538, 2.1971418857574463, 2.192904233932495, 2.206490993499756, 2.1846179962158203, 2.1788320541381836, 2.207704782485962, 2.201770305633545, 2.2023539543151855, 2.206277847290039, 2.2214746475219727, 2.2020552158355713, 2.1897895336151123, 2.188129425048828, 2.1837947368621826, 2.190734624862671, 2.1909642219543457, 2.1952972412109375, 2.191692590713501, 2.202016830444336, 2.1866466999053955, 2.2030296325683594, 2.183807849884033, 2.199331283569336, 2.185497522354126, 2.191472053527832};
  double chicomp;
  int np=0,ni=0;
  while(np<720)
  {
    if(!isinf(Ranges[np]))
    {
        break;

    }

  np++;

  }
  ni=np;
  kbs.push_back(ni);
  kf.update_if(ni);
  if(!isinf(Ranges[719])) 
    {kf.update_if(719);
    kbs.push_back(719);
    }

  while(np<720){
  if(ni==np)
  {
  x0 << Ranges[np], 0;

  y << Ranges[np];

  kf.calcp(Ranges[np]);
  kf.init(x0);

  }
  else
  {
    y << Ranges[np];
    chicomp = kf.update_pre(y); 
  
    if(chicomp>=3.84)
    {
      ni=np;
      np--;
      kf.update_if(np);
      kbs.push_back(np);
      if(isinf(Ranges[ni])){
        while(np<720){
                  np++;
          if(!isinf(Ranges[np])) break;

          
        }
      ni=np;
      kf.update_if(np);
     kbs.push_back(np);     
    np--;

      }
      else 
        {kf.update_if(ni);
        kbs.push_back(ni);
        }


    }

     else 
     {
      kf.update_else(y);
     }

  }

  np++;

  }

  //int len =0;
 //kf.printkb();
 /* for(int j=0;j<720;j++){
    if(kbs[j]==1){
pt1.x=xs[j];
pt1.y=ys[j];
pt1.z=0;
points.points.push_back(pt1);
len++;
    }
    }*/
      seg1.clear();

std::sort(kbs.begin(),kbs.end());
 for (std::vector<unsigned int>::const_iterator cit0 = kbs.begin(); cit0 != kbs.end(); ++cit0)
 {

if((*(cit0+1)-*cit0)>5 && *cit0!=719) {
  //find radius _angle 
 if (!isinf(Ranges[*cit0+6])){
    //ROS_DEBUG("Before segmentation %d %d",*cit0,*(cit0+1));

  split(*cit0,*(cit0+1));
}
}

 }


for(std::vector<unsigned int>::const_iterator cit1 = seg1.begin(); cit1 != seg1.end(); ++cit1){
  //ROS_DEBUG("%d",*cit1);
//int indx1 =seg1[*cit1];
  if(!isinf(xs[*cit1]) && !isinf(ys[*cit1])){
pt1.x=xs[*cit1];
pt1.y=ys[*cit1];  
pt1.z=0;
points.points.push_back(pt1);
 //marker_publ_.publish(points);
}

}
std::sort(seg1.begin(),seg1.end());

for(int h=0;h<seg1.size();h=h+2){
  //ROS_DEBUG("%d",*cit1);
//int indx1 =seg1[*cit1];
  if(!isinf(xs[seg1[h]]) && !isinf(ys[seg1[h]]) && !isinf(xs[seg1[h+1]]) && !isinf(ys[seg1[h+1]])){
pt2line.x=xs[seg1[h]];
pt2line.y=ys[seg1[h]];  
pt2line.z=0;

line_list.points.push_back(pt2line);
pt2line.x=xs[seg1[h+1]];
pt2line.y=ys[seg1[h+1]];  
pt2line.z=0;
ROS_DEBUG("After segmentation %d --------------------> %d",seg1[h],seg1[h+1]);
line_list.points.push_back(pt2line);

 //marker_publ_.publish(points);
}

}

marker_publ_.publish(points);
marker_publ_.publish(line_list);


}