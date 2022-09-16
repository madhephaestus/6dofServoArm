
import com.neuronrobotics.bowlerstudio.physics.TransformFactory
import com.neuronrobotics.sdk.addons.kinematics.AbstractKinematicsNR
import com.neuronrobotics.sdk.addons.kinematics.DHChain;
import com.neuronrobotics.sdk.addons.kinematics.DhInverseSolver;
import com.neuronrobotics.sdk.addons.kinematics.WristNormalizer
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR;
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR;

import java.text.DecimalFormat
import java.util.ArrayList;

import com.neuronrobotics.sdk.addons.kinematics.DHLink;
import com.neuronrobotics.sdk.common.Log;
import Jama.Matrix;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class scriptJavaIKModel implements DhInverseSolver {
	boolean debug = false;

	int limbIndex =0;
	public scriptJavaIKModel(int index){
		limbIndex=index;
	}
	@Override
	public double[] inverseKinematics(TransformNR target, double[] jointSpaceVector, DHChain chain) {
		ArrayList<DHLink> links = chain.getLinks();
		return inverseKinematics6dof(target,jointSpaceVector,chain);
	}
	TransformNR linkOffset(DHLink link) {
		return new TransformNR(link.DhStep(0))
	}
	public double[] inverseKinematics6dof(TransformNR target, double[] jointSpaceVector, DHChain chain) {

		//System.out.println("My 6dof IK "+target);
		double[] current = new double[jointSpaceVector.length]
		for(int i=0;i<current.length;i++) {
			current[i]=jointSpaceVector[i];
		}
		ArrayList<DHLink> links = chain.getLinks();
		int linkNum = jointSpaceVector.length;
		TransformNR l0Offset = linkOffset(links.get(0))
		TransformNR l1Offset = linkOffset(links.get(1))
		TransformNR l2Offset = linkOffset(links.get(2))
		TransformNR l3Offset = linkOffset(links.get(3))
		// Vector decompose the tip target
		double z = target.getZ();
		double y = target.getY();
		double x = target.getX();
		def targetNoRot =new TransformNR(x,y,z,new RotationNR())
		
		RotationNR q = target.getRotation();
		def newCenter =target.copy()
		// Start by finding the IK to the wrist center
		if(linkNum>=6) {
			//offset for tool
			if(debug)println "Offestting for tool"
			def tool = new TransformNR()
			if(linkNum==7)
				tool=linkOffset(links.get(6))
			// compute the transform from tip to wrist center
			def wristCenterOffsetTransform = linkOffset(links.get(5)).times(tool)
			//println wristCenterOffsetTransform
			// take off the tool from the target to get the center of the wrist
			newCenter = target.times(wristCenterOffsetTransform.inverse())

			//if(debug)Platform.runLater({TransformFactory.nrToAffine(newCenter,tipPointer2.getManipulator())})
		}
		def virtualcenter = newCenter.times(new TransformNR(0,0,10,
			 new RotationNR(Math.toDegrees(links.get(5).getAlpha()),0,0)))
		// recompute the X,y,z with the new center
		z = newCenter.getZ();
		y = newCenter.getY();
		x = newCenter.getX();
		//xyz now are at the wrist center
		// Compute the xy plane projection of the tip
		// this is the angle of the tipto the base link
		if(x==0&&y==0) {
			println "Singularity! try something else"
			return inverseKinematics6dof(target.copy().translateX(0.01),current,chain);
		}

		double baseVectorAngle = Math.atan2(y , x);
		double a1d = Math.toDegrees(baseVectorAngle);
		// this projection number becomes the base link angle directly
		jointSpaceVector[0]=a1d;
		if(debug)println "New base "+a1d
		//jointSpaceVector[0]=0;// TESTING

		// Rotate the tip into the xZ plane
		// apply a transform to the tip here to compute where it
		// would be on the ZX plane if the base angel was 0
		double alphaBase =
				Math.toDegrees(
				links.get(0).getAlpha()
				)
		def firstLink =new TransformNR(links.get(0).DhStep(baseVectorAngle)).inverse()
		def tipNoRot =new TransformNR(x,y,z,new RotationNR())

		//println "Incomming tip target Tip \tx="+x+" z="+z+" and y="+y+" alph baseLink "+alphaBase
		//println firstLink
		//println tipNoRot

		def newTip = firstLink
				.times(tipNoRot)

		x=newTip.getX()
		y=newTip.getY()
		z=newTip.getZ()
		if(x==0&&y==0) {
			println "Singularity! try something else"
			return inverseKinematics6dof(target.copy().translateX(0.01),current,chain);
		}
		if(debug)println "New Tip                             \tx="+x+" y="+y+" and z should be 0 and is="+z


		//println newTip
		// Tip y should be 0
		// this is the angle of the vector from base to tip
		double tipToBaseAngle = Math.atan2(y,x); // we now have the rest of the links in the XY plane
		
		double tipToBaseAngleDegrees = Math.toDegrees(tipToBaseAngle);
		if(debug)println "Base link to tip angle elevation "+tipToBaseAngleDegrees
		def transformAngleOfTipToTriangle = new TransformNR(0,0,0,new RotationNR(0,-tipToBaseAngleDegrees,0))
		def xyTip = new TransformNR(x,y,0,new RotationNR())
		//Transform the tip into the x Vector
		def tipXPlane =transformAngleOfTipToTriangle
				.times(xyTip)
		//println tipXYPlane
		double wristVect = tipXPlane.getX();
		// add together the last two links
		TransformNR wristCenterToElbow = 	l2Offset.times(l3Offset)//.inverse()
		// find the angle formed by the two links, includes the elbows theta
		if(wristCenterToElbow.getX()==0&&wristCenterToElbow.getY()==0) {
			println "Singularity! try something else"
			return inverseKinematics6dof(target.copy().translateX(0.01),current,chain);
		}
		double elbowLink2CompositeAngle = Math.atan2(wristCenterToElbow.getY(),wristCenterToElbow.getX());
		double elbowLink2CompositeAngleDegrees = Math.toDegrees(elbowLink2CompositeAngle)
		// COmpute teh vector length of the two links combined
		double elbowLink2CompositeLength = Math.sqrt(
				Math.pow(wristCenterToElbow.getY(), 2) +
				Math.pow(wristCenterToElbow.getX(), 2));
		// assume no D on this link as that would break everything
		double elbowLink1CompositeLength = links.get(1).getR();
		if(debug)println "Elbow 2 link data "+elbowLink2CompositeAngleDegrees+" vector "+elbowLink2CompositeLength

		if(wristVect>elbowLink2CompositeLength+elbowLink1CompositeLength)
			throw new ArithmeticException("Total reach longer than possible "+target);
		// Use the law of cosines to calculate the elbow and the shoulder tilt
		double shoulderTiltAngle =-( Math.toDegrees(Math.acos(
				(Math.pow(elbowLink1CompositeLength,2)+Math.pow(wristVect,2)-Math.pow(elbowLink2CompositeLength,2))/
				(2*elbowLink1CompositeLength*wristVect)
				))-tipToBaseAngleDegrees+Math.toDegrees(links.get(1).getTheta()))
		double elbowTiltAngle =-( Math.toDegrees(Math.acos(
				(Math.pow(elbowLink2CompositeLength,2)+Math.pow(elbowLink1CompositeLength,2)-Math.pow(wristVect,2))/
				(2*elbowLink2CompositeLength*elbowLink1CompositeLength)
				))+elbowLink2CompositeAngleDegrees-180)
		jointSpaceVector[2]=elbowTiltAngle
		jointSpaceVector[1]=shoulderTiltAngle
		
		
		/**
		// compute the top of the wrist now that the first 3 links are calculated
		 * 
		 */
		ArrayList<TransformNR> chainToLoad =[]
		chain.forwardKinematicsMatrix(jointSpaceVector,chainToLoad)
		def	startOfWristSet=chain.kin.inverseOffset(chainToLoad.get(2));
		TransformNR wristMOvedToCenter0 =startOfWristSet
											.inverse()// move back from base ot wrist to world home
											.times(virtualcenter)// move forward to target, leaving the angle between the tip and the start of the rotation 
		if(debug)println 	wristMOvedToCenter0								
		RotationNR qWrist=wristMOvedToCenter0.getRotation()
		if(wristMOvedToCenter0.getX()==0&&wristMOvedToCenter0.getY()==0) {
			println "Singularity! try something else"
			return inverseKinematics6dof(target.copy().translateX(0.01),current,chain);
		}
		def tmp = (Math.toDegrees(Math.atan2(wristMOvedToCenter0.getY(), wristMOvedToCenter0.getX()))-Math.toDegrees(links.get(3).getTheta()))
		def plus180 = tmp+180
		def minus180 = tmp-180
		def closest=tmp
		def options = [ tmp,plus180,minus180]
		def currentWristStart=jointSpaceVector[3]
		def lowestDelt = Math.abs(currentWristStart-tmp)
		for(def val:options) {
			def delt =  Math.abs(currentWristStart-val)
			if(delt<lowestDelt)
				closest=val
		}
		jointSpaceVector[3]=closest
		if(jointSpaceVector.length==4)
			return jointSpaceVector
		
		chainToLoad =[]
		/**
		// Calculte the second angle
		 * 
		 */
		chainToLoad.clear()
		chain.forwardKinematicsMatrix(jointSpaceVector,chainToLoad)
		def	startOfWristSet2=chain.kin.inverseOffset(chainToLoad.get(3));
		TransformNR wristMOvedToCenter1 =startOfWristSet2
											.inverse()// move back from base ot wrist to world home
											.times(virtualcenter)// move forward to target, leaving the angle between the tip and the start of the rotation
		if(debug)println " Middle link ="	+wristMOvedToCenter1
		RotationNR qWrist2=wristMOvedToCenter1.getRotation()
		if(wristMOvedToCenter1.getX()==0&&wristMOvedToCenter1.getY()==0) {
			println "Singularity! try something else"
			return inverseKinematics6dof(target.copy().translateX(0.01),current,chain);
		}
		jointSpaceVector[4]=(Math.toDegrees(Math.atan2(wristMOvedToCenter1.getY(), wristMOvedToCenter1.getX()))-
			Math.toDegrees(links.get(4).getTheta())+
			90)
		if(jointSpaceVector.length==5)
			return jointSpaceVector
		chainToLoad =[]
		/**
		// Calculte the last angle
		 * 
		 */
		chain.forwardKinematicsMatrix(jointSpaceVector,chainToLoad)
		def	startOfWristSet3=chain.kin.inverseOffset(chainToLoad.get(4));
		def tool = new TransformNR()
		if(linkNum==7)
			tool=linkOffset(links.get(6))
		TransformNR wristMOvedToCenter2 =startOfWristSet3
											.inverse()// move back from base ot wrist to world home
											.times(target.times(tool.inverse()))// move forward to target, leaving the angle between the tip and the start of the rotation
		if(debug)println "\n\nLastLink "	+wristMOvedToCenter2
		RotationNR qWrist3=wristMOvedToCenter2.getRotation()
		jointSpaceVector[5]=(Math.toDegrees(qWrist3.getRotationAzimuth())-Math.toDegrees(links.get(5).getTheta()))
		
		double[] j =[jointSpaceVector[3],jointSpaceVector[4],jointSpaceVector[5]]as double[];
		double[] c =	[current[3],current[4],current[5]]as double[]
		double[] nrm = normalize(
			j,
			c,
			chain);
//		double[] nrm = normalize(
//			j,
//			c,
//			chain);
		jointSpaceVector[3]=nrm[0]
		jointSpaceVector[4]=nrm[1]
		jointSpaceVector[5]=nrm[2]
		
		if(debug)println "Euler Decomposition proccesed \n"+jointSpaceVector[3]+" \n"+jointSpaceVector[4]+" \n"+jointSpaceVector[5]
		//println "Law of cosines results "+shoulderTiltAngle+" and "+elbowTiltAngle
		return jointSpaceVector;
	}
	
	double[] option(double w1,double w2,double w3) {
		return [w1,w2,w3] as double[]
	}
	
	double[] normalize(double[] calculated,double[] current, DHChain chain) {
		AbstractKinematicsNR kin = chain.kin;
		DecimalFormat df = new DecimalFormat("000.00");
		
		double[] alt1 =option( calculated[0] - 180, -calculated[1], calculated[2] - 180 );
		double[] calculated2 =option( calculated[0] + 360, calculated[1] + 360, calculated[2] + 360 );
		double[] calculated3 =option( calculated[0] - 360, calculated[1] - 360, calculated[2] - 360 );
		double[] alt2 =option( alt1[0] + 360, alt1[1] + 360, alt1[2] + 360 );
		double[] alt3 =option( alt1[0] - 360, alt1[1] - 360, alt1[2] - 360 );
		double[] calculated6 =option( calculated[0] - 360, calculated[1], calculated[2] );
		double[] calculated7 =option( calculated[0] + 360, calculated[1], calculated[2] );
		double[] als4 =option( alt1[0] - 360, alt1[1], alt1[2] );
		double[] alt5 =option( alt1[0] + 360, alt1[1], alt1[2] );
		
		HashMap<double[], Double> scores = new HashMap<>();
		score(calculated, current, scores, kin);
		score(alt1, current, scores, kin);
		score(calculated2, current, scores, kin);
		score(calculated3, current, scores, kin);
		score(alt2, current, scores, kin);
		score(alt3, current, scores, kin);
		score(calculated6, current, scores, kin);
		score(calculated7, current, scores, kin);
		score(als4, current, scores, kin);
		score(alt5, current, scores, kin);
		score(option( calculated[0] -180, -calculated[1], calculated[2]+180 ), current, scores, kin);
		score(option( alt1[0] -180, alt1[1], -alt1[2]+180 ), current, scores, kin);
		score(option( calculated[0] +180, -calculated[1], calculated[2]-180 ), current, scores, kin);
		score(option( alt1[0] +180, -alt1[1], alt1[2]-180 ), current, scores, kin);
		
		if(scores.size()>0) {
			double[] start =calculated ;
			if(scores.get(start)==null) {
				start = (double[]) scores.keySet().toArray()[0];
			}
			double score=scores.get(start);
			double[] ret=start;
			//println "\n\n"
			for(double[]  tmp:scores.keySet()) {
				double delt =scores.get(tmp)
				//println ""+tmp.collect{df.format(it)}+" score "+delt+" cur "+current.collect{df.format(it)}
				if(delt<score) {
					score=delt
					ret=tmp
					//println "Best yet"
				}
			}
	//		if(ret!=calculated)
	//			println "Current "+current.collect{df.format(it)}+" Normalizing wrist from:\n"+calculated.collect{df.format(it)+"\t"}+"\nto:\n"+ret.collect{df.format(it)+"\t"}
			return ret
		}
		//println("No Solution! "+calculated.collect{df.format(it)}+" cur "+current.collect{df.format(it)})
		//throw new RuntimeException("No Wrist Solution! ");
		return current
	}
	
	void score(double[] calculated,double[] current,HashMap<double[],Double> scores,AbstractKinematicsNR kin ) {
		double delt=0;
		for(int i=0;i<3;i++) {
			int i3 = i+3;
			calculated[i]=calculated[i] % 360;
//			if(calculated[i] >kin.getMaxEngineeringUnits(i3)) {
//				return //calculated[i]=kin.getMaxEngineeringUnits(i3);
//			}
//			if(calculated[i] <kin.getMinEngineeringUnits(i3)) {
//				return //calculated[i]=kin.getMinEngineeringUnits(i3)
//			}
			double measure = current[i]-calculated[i];
			if(Math.abs(measure)>Math.abs(delt)) {
				delt=measure;
			}
		}
		scores.put(calculated, Math.abs(delt));
	}
	

}

if(args==null)
	args=[0]
return new scriptJavaIKModel (args[0])
