package org.firstinspires.ftc.teamcode.robotParts;

public class Vector {
    private  double[] comps;
    private double mag;

    public Vector(double xCom, double yCom){
        this.comps = new double[2];
        this.comps[0] = xCom;
        this.comps[1] = yCom;
        this.mag = Math.hypot(xCom, yCom);
    }
    public Vector(){
        this.comps = new double[2];
        this.comps[0] = 0;
        this.comps[1] = 0;
        this.mag = Math.hypot(0, 0);
    }

    public Vector addition(Vector otherVec){
        return new Vector(this.comps[0]+otherVec.comps[0],this.comps[1]+otherVec.comps[1]);
    }
    public Vector subtraction(Vector otherVec){
        return new Vector(this.comps[0]-otherVec.comps[0],this.comps[1]-otherVec.comps[1]);
    }
    public void ScalarMult(double scalar){
        this.comps[0] = this.comps[0] * scalar;
        this.comps[1] = this.comps[1] * scalar;
    }
    public double dotProduct(Vector otherVec) {
        return ((this.comps[0]*otherVec.comps[0])+(this.comps[1]*otherVec.comps[1]));
    }
    public void makeUnit(){
        this.comps[0] = this.comps[0]/this.mag;
        this.comps[1] = this.comps[1]/this.mag;
    }
    public double theta(){
        return Math.atan2(this.comps[1],this.comps[0]);

    }

    public double getMag() {
        return mag;
    }

    public void setComps(double[] comps) {
        this.comps = comps;
        this.mag = Math.hypot(this.comps[0], this.comps[1]);
    }

    public double[] getComps() {
        return comps;
    }
}
