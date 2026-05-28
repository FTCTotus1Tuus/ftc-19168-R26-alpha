package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

public final class DriveHardwareResolver {

    private DriveHardwareResolver() {
    }

    private static final String[] LEFT_FRONT_CANDIDATES = {
            RobotHardwareNames.LEFT_FRONT_MOTOR,
            "leftFront",
            "frontLeft",
            "left_front",
            "lf"
    };

    private static final String[] LEFT_REAR_CANDIDATES = {
            RobotHardwareNames.LEFT_REAR_MOTOR,
            "leftRear",
            "rearLeft",
            "left_rear",
            "lr"
    };

    private static final String[] RIGHT_FRONT_CANDIDATES = {
            RobotHardwareNames.RIGHT_FRONT_MOTOR,
            "rightFront",
            "frontRight",
            "right_front",
            "rf"
    };

    private static final String[] RIGHT_REAR_CANDIDATES = {
            RobotHardwareNames.RIGHT_REAR_MOTOR,
            "rightRear",
            "rearRight",
            "right_rear",
            "rr"
    };

    public static Result resolve(HardwareMap hardwareMap) {
        List<String> available = listMotorNames(hardwareMap);
        Set<String> availableSet = new LinkedHashSet<>(available);
        Set<String> used = new LinkedHashSet<>();

        String leftFront = pick(availableSet, used, LEFT_FRONT_CANDIDATES, "left", "front");
        String leftRear = pick(availableSet, used, LEFT_REAR_CANDIDATES, "left", "rear");
        String rightFront = pick(availableSet, used, RIGHT_FRONT_CANDIDATES, "right", "front");
        String rightRear = pick(availableSet, used, RIGHT_REAR_CANDIDATES, "right", "rear");

        StringBuilder notes = new StringBuilder();
        if (leftFront == null) notes.append("Missing left-front motor name. ");
        if (leftRear == null) notes.append("Missing left-rear motor name. ");
        if (rightFront == null) notes.append("Missing right-front motor name. ");
        if (rightRear == null) notes.append("Missing right-rear motor name. ");

        return new Result(leftFront, leftRear, rightFront, rightRear, available, notes.toString().trim());
    }

    private static String pick(Set<String> available, Set<String> used, String[] preferred, String sideHint, String endHint) {
        for (String candidate : preferred) {
            String match = findCaseInsensitive(available, candidate);
            if (match != null && !used.contains(match)) {
                used.add(match);
                return match;
            }
        }

        for (String name : available) {
            if (used.contains(name)) {
                continue;
            }
            String lowered = name.toLowerCase(Locale.US);
            if (lowered.contains(sideHint) && lowered.contains(endHint)) {
                used.add(name);
                return name;
            }
        }

        return null;
    }

    private static String findCaseInsensitive(Set<String> available, String expected) {
        for (String name : available) {
            if (name.equalsIgnoreCase(expected)) {
                return name;
            }
        }
        return null;
    }

    public static List<String> listMotorNames(HardwareMap hardwareMap) {
        LinkedHashSet<String> names = new LinkedHashSet<>();
        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            names.addAll(hardwareMap.getNamesOf(motor));
        }
        return new ArrayList<>(names);
    }

    public static final class Result {
        public final String leftFront;
        public final String leftRear;
        public final String rightFront;
        public final String rightRear;
        public final List<String> availableMotorNames;
        public final String notes;

        Result(String leftFront, String leftRear, String rightFront, String rightRear, List<String> availableMotorNames, String notes) {
            this.leftFront = leftFront;
            this.leftRear = leftRear;
            this.rightFront = rightFront;
            this.rightRear = rightRear;
            this.availableMotorNames = availableMotorNames;
            this.notes = notes;
        }

        public boolean isComplete() {
            return leftFront != null && leftRear != null && rightFront != null && rightRear != null;
        }
    }
}

