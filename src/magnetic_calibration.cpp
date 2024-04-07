//
// Created by lemezoth on 29/02/24.
// Updated by godardma on 04/04/24.
//

#include "magnetic_calibration/magnetic_calibration.h"

#include <iostream>
#include <vector>

#include "magnetic_calibration/magnetic_paving.h"

#include <vtkSmartPointer.h>
#include <vtkCamera.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSphereSource.h>

#include <vtkChartXYZ.h>
#include <vtkContext3D.h>
#include <vtkContextScene.h>
#include <vtkContextView.h>
#include <vtkFloatArray.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPen.h>
#include <vtkPlotLine3D.h>
#include <vtkTable.h>
#include <vtkGlyph3D.h>
#include <vtkAppendPolyData.h>

#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>

#include <vtkParametricEllipsoid.h>
#include <vtkParametricFunctionSource.h>

#include <vtkNamedColors.h>
#include <vtkProperty.h>

#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>


void MagneticCalibration::generate_data() {
    magnetometer_data_= { 
        {0.9574534627629101,3.794951438334881,2.5119287517843056},
        {1.9508903958740524,3.6964742020015335,2.619320319326598},
        {2.881053704148721,3.6369270381490066,2.362519900218152},
        {3.5135195575083698,3.474864727245879,2.052671405548784},
        {3.934190915686245,3.3014997997753524,1.7092484452510044},
        {3.8789137909227143,3.140008364147525,1.5184448326205124},
        {3.5874261074771545,2.9012802786670875,1.2484873450917073},
        {2.908838115210677,2.840911786594086,0.714605315486504},
        {1.9387122764092577,2.7995655819821446,0.7289429567484075},
        {0.8424997647725712,2.6972173104644743,0.8003344507043961},
        {1.1591506373299643,3.828189525418029,2.2947430718339135},
        {1.8999035485461684,4.075122385997126,2.164802472911838},
        {2.53988169814714,4.253356264610823,1.7668584875733329},
        {2.84681157165224,4.337321125964499,1.4935420740387282},
        {3.2261790970727584,4.54423304530482,1.116763933327159},
        {3.2377229154834364,4.023677054672885,0.8382402813295914},
        {3.175963751104408,3.8766233639613463,0.6837112526121536},
        {2.4765821450691035,3.7121905084206546,0.5502243680903368},
        {1.6596638715676562,3.188100374271366,0.5080629015498299},
        {1.140930021664503,2.8029367054603047,0.8885030049414193},
        {0.9734421848771563,3.661983584533347,2.471100182619671},
        {1.250970848790685,4.309944544719245,2.252776490335085},
        {1.2504442582364739,4.8612049714407135,1.7083780095881946},
        {1.4074737686164238,4.898955761904579,1.145653157787963},
        {1.6732956648871322,4.919252546666402,0.7094041136446351},
        {1.393029633823517,4.878454097120544,0.4716031244987363},
        {1.5400913427205585,4.356116892556202,0.30233291249953914},
        {1.3067946044709717,4.0212728354235905,0.426588647329578},
        {1.1170227105533588,3.424978492898474,0.33409857412845334},
        {0.961324722723642,2.6842751781215686,0.5992195323852267},
        {0.8912710762796249,3.8499365101217764,2.5594466929243835},
        {0.5129412024212155,3.9858657682108887,2.2111342398718734},
        {0.2662520133207608,4.521386602470743,1.6902576600310055},
        {-0.21783135863433326,4.755479468975863,1.3278500036066365},
        {-0.44869263155293615,4.807715158972324,1.003264243013484},
        {-0.3639324266551419,4.708050593938462,0.6642752888532052},
        {-0.35884370722296405,4.432354961630075,0.4531618570969025},
        {0.10361046238877854,3.6435597750725894,0.3856564805677569},
        {0.5301097226133679,3.3462950843591885,0.4771895305006676},
        {0.8949681096858099,2.8032321566541136,0.9464255780427788},
        {0.9393125628272138,3.743967020744614,2.4172889097806918},
        {0.22817776528162692,3.96333904326743,2.2598732919291855},
        {-0.8618138276684995,4.021896706010826,2.0840207698639026},
        {-1.369858158940907,4.059976595101915,1.7671348923788734},
        {-1.7557175479068052,3.7906114995496525,1.496781377639512},
        {-1.8061812806700515,3.7509233603707464,1.0706812916525243},
        {-1.3071876386484131,3.3213860373079975,0.8670575874439624},
        {-0.9436980394916745,3.305526968736601,0.7678768971657479},
        {-0.018617392584144354,3.042284346017636,0.7477866735617196},
        {1.0716811936341686,2.6733587675642108,0.7427592909709295},
        {1.0643153335375457,3.7788581479820618,2.4996264882575003},
        {0.1445921430734308,3.4387095037106397,2.609518645196045},
        {-0.8955288453194856,3.3435927891158355,2.450020988040842},
        {-1.4161995143144523,2.8109380420916783,2.2175964313450254},
        {-1.6618082362257278,2.804081190828959,1.8904230834159765},
        {-1.7987768781171996,2.5596941049347435,1.9175762575432054},
        {-1.5677560087254807,2.5064138044785937,1.5981364762330896},
        {-0.7296022613540876,2.49882620325898,1.3331250540859856},
        {-0.0976475826258962,2.7740163022371145,1.0251894957450671},
        {0.8349861032950537,2.6423619167864207,0.8306274273853089},
        {0.9681389111961067,3.8164424382354714,2.6145108380191404},
        {0.5774798567553251,3.161707717095309,2.671777535197263},
        {-0.14532857138915198,2.697556818717823,3.0602537040311013},
        {-0.15930230221267452,2.306918457262381,2.7261043308047825},
        {-0.5876787464455582,1.8992984686179126,2.3730717795997838},
        {-0.5455639803929763,1.576218285504646,2.1659485571085977},
        {-0.3430937492321378,1.404646917510683,2.0466453050206734},
        {-0.13556090412855681,1.894064839624247,1.4210534967812793},
        {0.5212590534792753,2.153197383994328,1.0900081813600482},
        {1.0320296933233442,2.7247066388952597,0.8122722058394921},
        {1.0467734185989008,3.6497173914149537,2.4949851844742645},
        {1.0022831227117737,2.9867993400769968,2.5694743190661864},
        {1.394338856059465,2.3501246442890342,2.828022319923921},
        {1.5415323626512172,1.9559065919612657,3.020126785434969},
        {1.7452096313608174,1.6052206682930437,2.7044549127574182},
        {1.428838568781103,1.2789705769174802,2.344030323472697},
        {1.3957269220756408,1.4687880353694358,1.9382378054305842},
        {1.2061803008315768,1.785808567357935,1.4678218945602022},
        {1.2772794054644776,2.185351097200632,0.9993205344136155},
        {1.103374431479584,2.6440460940251582,0.456689250784516},
        {0.8904144663919135,3.8084350465673555,2.2642699714977597},
        {1.8126123268773018,3.2249902123307708,2.690144188965926},
        {2.4417872442429855,2.8593809744805814,2.7675965416729507},
        {2.7998031209580008,2.5192025391391217,2.6339433173255893},
        {3.1369777755465575,2.239043669897273,2.353266591623633},
        {3.2832363011180625,1.9986443215813563,2.1454910104643856},
        {3.02800614800259,1.984301459692805,1.6604428988889326},
        {2.5727929702250143,2.063456910323275,1.423995289906275},
        {1.7206323738835334,2.5684303833325814,0.9559623643239666},
        {1.1940610269118705,2.5653396709098586,0.7204395911925745},
        {1.0254676488330703,3.934131441928424,2.5122126319522247},
        {2.117730131495122,3.7402441438104885,2.478426840147806},
        {2.8553090101777396,3.646096333063374,2.203687561561154},
        {3.7403671679035337,3.502338949278852,2.0888378610154934},
        {3.7936157474636207,3.228649831127796,1.82310690055221},
        {3.9888735882243185,3.0070593000947756,1.3217938832808576},
        {3.3943916937589558,3.074643504389761,1.335160932922825},
        {2.868977794043375,2.9018270259148267,0.8430436321633901},
        {2.076402007938496,2.969697779630358,0.8243252262148255},
        {0.9474539118022715,2.8458993004750406,0.7015903566941875},
        };

        magnetometer_data_regularized_ = { 
        {0.9574534627629101,3.794951438334881,2.5119287517843056},
        {1.9508903958740524,3.6964742020015335,2.619320319326598},
        {2.881053704148721,3.6369270381490066,2.362519900218152},
        {3.5135195575083698,3.474864727245879,2.052671405548784},
        {3.934190915686245,3.3014997997753524,1.7092484452510044},
        {3.8789137909227143,3.140008364147525,1.5184448326205124},
        {3.5874261074771545,2.9012802786670875,1.2484873450917073},
        {2.908838115210677,2.840911786594086,0.714605315486504},
        {1.9387122764092577,2.7995655819821446,0.7289429567484075},
        {0.8424997647725712,2.6972173104644743,0.8003344507043961},
        {1.1591506373299643,3.828189525418029,2.2947430718339135},
        {1.8999035485461684,4.075122385997126,2.164802472911838},
        {2.53988169814714,4.253356264610823,1.7668584875733329},
        {2.84681157165224,4.337321125964499,1.4935420740387282},
        {3.2261790970727584,4.54423304530482,1.116763933327159},
        {3.2377229154834364,4.023677054672885,0.8382402813295914},
        {3.175963751104408,3.8766233639613463,0.6837112526121536},
        {2.4765821450691035,3.7121905084206546,0.5502243680903368},
        {1.6596638715676562,3.188100374271366,0.5080629015498299},
        {1.140930021664503,2.8029367054603047,0.8885030049414193},
        {0.9734421848771563,3.661983584533347,2.471100182619671},
        {1.250970848790685,4.309944544719245,2.252776490335085},
        {1.2504442582364739,4.8612049714407135,1.7083780095881946},
        {1.4074737686164238,4.898955761904579,1.145653157787963},
        {1.6732956648871322,4.919252546666402,0.7094041136446351},
        {1.393029633823517,4.878454097120544,0.4716031244987363},
        {1.5400913427205585,4.356116892556202,0.30233291249953914},
        {1.3067946044709717,4.0212728354235905,0.426588647329578},
        {1.1170227105533588,3.424978492898474,0.33409857412845334},
        {0.961324722723642,2.6842751781215686,0.5992195323852267},
        {0.8912710762796249,3.8499365101217764,2.5594466929243835},
        {0.5129412024212155,3.9858657682108887,2.2111342398718734},
        {0.2662520133207608,4.521386602470743,1.6902576600310055},
        {-0.21783135863433326,4.755479468975863,1.3278500036066365},
        {-0.44869263155293615,4.807715158972324,1.003264243013484},
        {-0.3639324266551419,4.708050593938462,0.6642752888532052},
        {-0.35884370722296405,4.432354961630075,0.4531618570969025},
        {0.10361046238877854,3.6435597750725894,0.3856564805677569},
        {0.5301097226133679,3.3462950843591885,0.4771895305006676},
        {0.8949681096858099,2.8032321566541136,0.9464255780427788},
        {0.9393125628272138,3.743967020744614,2.4172889097806918},
        {0.22817776528162692,3.96333904326743,2.2598732919291855},
        {-0.8618138276684995,4.021896706010826,2.0840207698639026},
        {-1.369858158940907,4.059976595101915,1.7671348923788734},
        {-1.7557175479068052,3.7906114995496525,1.496781377639512},
        {-1.8061812806700515,3.7509233603707464,1.0706812916525243},
        {-1.3071876386484131,3.3213860373079975,0.8670575874439624},
        {-0.9436980394916745,3.305526968736601,0.7678768971657479},
        {-0.018617392584144354,3.042284346017636,0.7477866735617196},
        {1.0716811936341686,2.6733587675642108,0.7427592909709295},
        {1.0643153335375457,3.7788581479820618,2.4996264882575003},
        {0.1445921430734308,3.4387095037106397,2.609518645196045},
        {-0.8955288453194856,3.3435927891158355,2.450020988040842},
        {-1.4161995143144523,2.8109380420916783,2.2175964313450254},
        {-1.6618082362257278,2.804081190828959,1.8904230834159765},
        {-1.7987768781171996,2.5596941049347435,1.9175762575432054},
        {-1.5677560087254807,2.5064138044785937,1.5981364762330896},
        {-0.7296022613540876,2.49882620325898,1.3331250540859856},
        {-0.0976475826258962,2.7740163022371145,1.0251894957450671},
        {0.8349861032950537,2.6423619167864207,0.8306274273853089},
        {0.9681389111961067,3.8164424382354714,2.6145108380191404},
        {0.5774798567553251,3.161707717095309,2.671777535197263},
        {-0.14532857138915198,2.697556818717823,3.0602537040311013},
        {-0.15930230221267452,2.306918457262381,2.7261043308047825},
        {-0.5876787464455582,1.8992984686179126,2.3730717795997838},
        {-0.5455639803929763,1.576218285504646,2.1659485571085977},
        {-0.3430937492321378,1.404646917510683,2.0466453050206734},
        {-0.13556090412855681,1.894064839624247,1.4210534967812793},
        {0.5212590534792753,2.153197383994328,1.0900081813600482},
        {1.0320296933233442,2.7247066388952597,0.8122722058394921},
        {1.0467734185989008,3.6497173914149537,2.4949851844742645},
        {1.0022831227117737,2.9867993400769968,2.5694743190661864},
        {1.394338856059465,2.3501246442890342,2.828022319923921},
        {1.5415323626512172,1.9559065919612657,3.020126785434969},
        {1.7452096313608174,1.6052206682930437,2.7044549127574182},
        {1.428838568781103,1.2789705769174802,2.344030323472697},
        {1.3957269220756408,1.4687880353694358,1.9382378054305842},
        {1.2061803008315768,1.785808567357935,1.4678218945602022},
        {1.2772794054644776,2.185351097200632,0.9993205344136155},
        {1.103374431479584,2.6440460940251582,0.456689250784516},
        {0.8904144663919135,3.8084350465673555,2.2642699714977597},
        {1.8126123268773018,3.2249902123307708,2.690144188965926},
        {2.4417872442429855,2.8593809744805814,2.7675965416729507},
        {2.7998031209580008,2.5192025391391217,2.6339433173255893},
        {3.1369777755465575,2.239043669897273,2.353266591623633},
        {3.2832363011180625,1.9986443215813563,2.1454910104643856},
        {3.02800614800259,1.984301459692805,1.6604428988889326},
        {2.5727929702250143,2.063456910323275,1.423995289906275},
        {1.7206323738835334,2.5684303833325814,0.9559623643239666},
        {1.1940610269118705,2.5653396709098586,0.7204395911925745},
        {1.0254676488330703,3.934131441928424,2.5122126319522247},
        {2.117730131495122,3.7402441438104885,2.478426840147806},
        {2.8553090101777396,3.646096333063374,2.203687561561154},
        {3.7403671679035337,3.502338949278852,2.0888378610154934},
        {3.7936157474636207,3.228649831127796,1.82310690055221},
        {3.9888735882243185,3.0070593000947756,1.3217938832808576},
        {3.3943916937589558,3.074643504389761,1.335160932922825},
        {2.868977794043375,2.9018270259148267,0.8430436321633901},
        {2.076402007938496,2.969697779630358,0.8243252262148255},
        {0.9474539118022715,2.8458993004750406,0.7015903566941875},
        };

}



void MagneticCalibration::regularize_data(){
    // Regularize the data
    MagneticPaving magnetic_paving(magnetometer_data_, 30, 1.0);
    magnetic_paving.process_data(magnetometer_data_regularized_);

    // Display min max
    magnetic_paving.cout_bounds();

    cout << "Regularized " << magnetometer_data_regularized_.size() << " values" << endl;
}


void MagneticCalibration::compute_ellipsoid() {
    // Compute the ellipsoid
    MatrixXd data(magnetometer_data_regularized_.size(), 3);
    for (size_t i = 0; i < magnetometer_data_regularized_.size(); ++i)
        for (size_t j = 0; j < 3; ++j)
            data(i, j) = magnetometer_data_regularized_[i][j];

    // http://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit

    // Create D matrix
    MatrixXd D(data.rows(), 9);
    D << data.col(0).array().square() + data.col(1).array().square() - 2 * data.col(2).array().square(),
            data.col(0).array().square() + data.col(2).array().square() - 2 * data.col(1).array().square(),
            2 * data.col(0).array() * data.col(1).array(),
            2 * data.col(0).array() * data.col(2).array(),
            2 * data.col(1).array() * data.col(2).array(),
            2 * data.col(0).array(),
            2 * data.col(1).array(),
            2 * data.col(2).array(),
            VectorXd::Constant(data.rows(), 1.0);
    D.transposeInPlace();

    VectorXd d2 = (data.rowwise().squaredNorm()).transpose();

    // Solve the linear system (several function are possible to compute the pseudo inverse)
    VectorXd u = (D * D.transpose()).jacobiSvd(ComputeThinU | ComputeThinV).solve(D * d2);

    auto a = VectorXd::Constant(1, u[0] + 1 * u[1] - 1);
    auto b = VectorXd::Constant(1, u[0] - 2 * u[1] - 1);
    auto c = VectorXd::Constant(1, u[1] - 2 * u[0] - 1);

    VectorXd v(10);
    v << a[0], b[0], c[0], u[2], u[3], u[4], u[5], u[6], u[7], u[8];

    MatrixXd A(4, 4);
    A << v[0], v[3], v[4], v[6],
            v[3], v[1], v[5], v[7],
            v[4], v[5], v[2], v[8],
            v[6], v[7], v[8], v[9];

    // Compute the center of the ellipsoid
    center_=(-A.topLeftCorner<3, 3>()).colPivHouseholderQr().solve(Vector3d(v[6], v[7], v[8]));

    // Compute the transformation matrix T
    Matrix4d T = Matrix4d::Identity();
    T.row(3).head(3) = center_;

    // Compute the rotated ellipsoid
    R_ = T * A * T.transpose();

    SelfAdjointEigenSolver<Matrix3d> eigen_solver(R_.topLeftCorner<3, 3>() / -R_(3, 3));
    Vector3d evals_ = eigen_solver.eigenvalues();
    Matrix3d evecs_ = eigen_solver.eigenvectors();


    radii_ = (1.0 / evals_.array().sqrt()).matrix();
    r_ = std::cbrt(radii_.prod());
    Matrix3d D1 = r_ * radii_.asDiagonal().inverse();

    TR_ = evecs_.transpose() * D1 * evecs_;
    cout<<"====================="<<endl;
    cout<<"Computing correction"<<endl << endl;
    cout << "Ellipsoid center: " << endl << center_.transpose() << endl << endl;
    cout << "Ellipsoid TR: " << endl << TR_ << endl << endl;
    ea_ = TR_.eulerAngles(0, 1, 2); 


}

void MagneticCalibration::correct_data(){
    // correct the data
    for (auto & data : magnetometer_data_){
        Vector3d data_eigen (data[0], data[1], data[2]);
        auto corrected_data=TR_*(data_eigen-center_);
        magnetometer_data_corrected_.push_back({corrected_data[0], corrected_data[1],corrected_data[2]});
    }
        
}

void MagneticCalibration::check_correction() {
    // Compute the ellipsoid
    cout<<"====================="<<endl;
    cout<<"Checking correction"<<endl << endl;
    MatrixXd data(magnetometer_data_corrected_.size(), 3);
    for (size_t i = 0; i < magnetometer_data_corrected_.size(); ++i)
        for (size_t j = 0; j < 3; ++j)
            data(i, j) = magnetometer_data_corrected_[i][j];

    // Create D matrix
    MatrixXd D(data.rows(), 9);
    D << data.col(0).array().square() + data.col(1).array().square() - 2 * data.col(2).array().square(),
            data.col(0).array().square() + data.col(2).array().square() - 2 * data.col(1).array().square(),
            2 * data.col(0).array() * data.col(1).array(),
            2 * data.col(0).array() * data.col(2).array(),
            2 * data.col(1).array() * data.col(2).array(),
            2 * data.col(0).array(),
            2 * data.col(1).array(),
            2 * data.col(2).array(),
            VectorXd::Constant(data.rows(), 1.0);
    D.transposeInPlace();

    VectorXd d2 = (data.rowwise().squaredNorm()).transpose();

    // Solve the linear system (several function are possible to compute the pseudo inverse)
    VectorXd u = (D * D.transpose()).jacobiSvd(ComputeThinU | ComputeThinV).solve(D * d2);


    auto a = VectorXd::Constant(1, u[0] + 1 * u[1] - 1);
    auto b = VectorXd::Constant(1, u[0] - 2 * u[1] - 1);
    auto c = VectorXd::Constant(1, u[1] - 2 * u[0] - 1);

    VectorXd v(10);
    v << a[0], b[0], c[0], u[2], u[3], u[4], u[5], u[6], u[7], u[8];

    MatrixXd A(4, 4);
    A << v[0], v[3], v[4], v[6],
            v[3], v[1], v[5], v[7],
            v[4], v[5], v[2], v[8],
            v[6], v[7], v[8], v[9];

    // Compute the center of the ellipsoid
    Vector3d center =(-A.topLeftCorner<3, 3>()).colPivHouseholderQr().solve(Vector3d(v[6], v[7], v[8]));

    // Compute the transformation matrix T
    Matrix4d T = Matrix4d::Identity();
    T.row(3).head(3) = center;

    // Compute the rotated ellipsoid
    Matrix4d R = T * A * T.transpose();

    SelfAdjointEigenSolver<Matrix3d> eigen_solver(R.topLeftCorner<3, 3>() / -R_(3, 3));
    Vector3d evals = eigen_solver.eigenvalues();
    Matrix3d evecs = eigen_solver.eigenvectors();

    Vector3d radii = (1.0 / evals.array().sqrt()).matrix();
    r_corrected_ = std::cbrt(radii.prod());
    Matrix3d D1 = r_corrected_ * radii.asDiagonal().inverse();

    Matrix3d TR = evecs.transpose() * D1 * evecs;
    cout << "Ellipsoid TR should give: " << endl << 1<<"    "<<0<<"    "<<0 << endl<< 0<<"    "<<1<<"    "<<0 <<"    "<< endl<< 0<<"    "<<0<<"    "<<1<<"    "<< endl << endl;
    cout << "Ellipsoid TR: " << endl << TR << endl << endl;
    cout<<endl;
    cout << "Ellipsoid center should give: " << endl << 0<<"    "<<0<<"    "<<0 << endl << endl;
    cout << "Ellipsoid center: " << endl << center.transpose() << endl << endl;



}

vtkSmartPointer<vtkActor> MagneticCalibration::generate_point_cloud(std::vector<std::array<double, 3>> &pts_data, const string &color, const double &radius){
    vtkNew<vtkNamedColors> colors;

    // Create a vtkPoints object to store the points
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    // Add points to vtkPoints object
    for (const auto& point : pts_data) {
        points->InsertNextPoint(point.data());
    }

    // Create a polydata to hold the points
    vtkSmartPointer<vtkPolyData> polydata_points = vtkSmartPointer<vtkPolyData>::New();
    polydata_points->SetPoints(points);

    // Create sphere source to represent the points
    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetRadius(radius); // Set sphere radius
    sphereSource->SetPhiResolution(10);
    sphereSource->SetThetaResolution(10);

    // Create glyph filter
    vtkSmartPointer<vtkGlyph3D> glyphFilter = vtkSmartPointer<vtkGlyph3D>::New();
    glyphFilter->SetInputData(polydata_points);
    glyphFilter->SetSourceConnection(sphereSource->GetOutputPort());
    glyphFilter->SetScaleFactor(1.0); // Set scale factor if needed

    // Create mapper and actor for combined data
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(glyphFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(colors->GetColor3d(color).GetData());

    return actor;
}

vtkSmartPointer<vtkMatrix4x4> eigenToVtkMatrix(const Eigen::Matrix4d& eigenMatrix) {
    vtkSmartPointer<vtkMatrix4x4> vtkMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            vtkMatrix->SetElement(i, j, eigenMatrix(i, j));
        }
    }
    return vtkMatrix;
}

vtkSmartPointer<vtkActor> MagneticCalibration::generate_ellipsoid(const string &color){
    vtkNew<vtkNamedColors> colors;

    // Create a parametric ellipsoid
    vtkSmartPointer<vtkParametricEllipsoid> ellipsoid = vtkSmartPointer<vtkParametricEllipsoid>::New();
    //  inflation
    ellipsoid->SetXRadius(radii_[0]);
    ellipsoid->SetYRadius(radii_[1]);
    ellipsoid->SetZRadius(radii_[2]);
    // Transform the ellipsoid
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    // Translation
    transform->Translate(center_[0],center_[1], center_[2]);
    // Rotation (before translation)
    transform->RotateX(ea_[0]*180.0/M_PI); 
    transform->RotateY(ea_[1]*180.0/M_PI);
    transform->RotateZ(ea_[2]*180.0/M_PI);
    // transform->Concatenate(eigenToVtkMatrix(R_));

    // Create a parametric function source and set the ellipsoid as the function
    vtkSmartPointer<vtkParametricFunctionSource> ellipsoidSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
    ellipsoidSource->SetParametricFunction(ellipsoid);
    ellipsoidSource->Update();

    // // Create a transform filter
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputData(ellipsoidSource->GetOutput());
    transformFilter->SetTransform(transform);
    transformFilter->Update();

    // // Create mapper and actor for combined data
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(transformFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(colors->GetColor3d(color).GetData());
    actor->GetProperty()->SetAmbient(1.);
    return actor;
}


vtkSmartPointer<vtkActor> MagneticCalibration::generate_unit_sphere(const string &color){
    vtkNew<vtkNamedColors> colors;

    // Create a parametric ellipsoid
    vtkSmartPointer<vtkParametricEllipsoid> ellipsoid = vtkSmartPointer<vtkParametricEllipsoid>::New();
    //  inflation
    ellipsoid->SetXRadius(r_corrected_);
    ellipsoid->SetYRadius(r_corrected_);
    ellipsoid->SetZRadius(r_corrected_);
    // Transform the ellipsoid
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

    // Create a parametric function source and set the ellipsoid as the function
    vtkSmartPointer<vtkParametricFunctionSource> ellipsoidSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
    ellipsoidSource->SetParametricFunction(ellipsoid);
    ellipsoidSource->Update();

    // // Create a transform filter
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputData(ellipsoidSource->GetOutput());
    transformFilter->SetTransform(transform);
    transformFilter->Update();

    // // Create mapper and actor for combined data
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(transformFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(colors->GetColor3d(color).GetData());
    actor->GetProperty()->SetAmbient(1.);
    actor->GetProperty()->SetOpacity(0.7);

    return actor;
}

void MagneticCalibration::view_data(float point_size=1.0,float axis_length=100.){
    vtkSmartPointer<vtkActor> actor_ellipsoid = generate_unit_sphere("Green");
    vtkSmartPointer<vtkActor> actor_regularized = generate_point_cloud(magnetometer_data_regularized_, "Red", point_size);
    vtkSmartPointer<vtkActor> actor_corrected = generate_point_cloud(magnetometer_data_corrected_, "Green",point_size);
    vtkSmartPointer<vtkActor> actor_raw = generate_point_cloud(magnetometer_data_, "Black",point_size/3.0);


    // Create renderer, render window, and interactor
    vtkSmartPointer<vtkCamera> sharedCamera = vtkSmartPointer<vtkCamera>::New();
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetActiveCamera(sharedCamera);
    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->SetSize(1000, 1000);
    renderWindow->SetWindowName("Magnetic Calibration");
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    // Add actor to the renderer
    renderer->AddActor(actor_corrected);
    renderer->AddActor(actor_regularized);
    renderer->AddActor(actor_raw);
    renderer->AddActor(actor_ellipsoid);

    renderer->SetBackground(0.1, 0.2, 0.4); // Set background color

    // Create axes
    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetTotalLength(axis_length,axis_length,axis_length);
    renderer->AddActor(axes);

    renderer->ResetCamera();
    sharedCamera->SetFocalPoint(0, 0, 0);

    // Start the interactor
    renderWindow->Render();
    renderWindowInteractor->Start();  // Start the event loop
}

int main(int argc, char *argv[]) {
    MagneticCalibration magnetic_calibration;
    magnetic_calibration.generate_data();
    magnetic_calibration.regularize_data();
    magnetic_calibration.compute_ellipsoid();
    magnetic_calibration.correct_data();
    magnetic_calibration.check_correction();
    magnetic_calibration.view_data(0.3,10.0);


    return 0;
}
