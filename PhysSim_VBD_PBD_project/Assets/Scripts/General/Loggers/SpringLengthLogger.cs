using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

public class SpringLengthLogger : MonoBehaviour
{
    public bool writeCsv = true;
    public bool showOverlay = true;
    public bool writePerSpring = true; // false logs only the summary columns (cloth has too many springs to dump per-spring)
    public string label = "Sim";
    public float overlayY = 110f;

    public Func<SpringLengthSample> Sampler { get; set; }

    private StreamWriter csv;
    private string csvPath;
    private float simTime;
    private int frameNumber;
    private SpringLengthSample latest;
    private bool headerWritten;
    private GUIStyle style;

    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    void Start()
    {
        if (writeCsv) OpenCsv();
    }

    void OnDestroy() { CloseCsv(); }
    void OnApplicationQuit() { CloseCsv(); }

    // Records the state before any stepping so the initial configuration
    // (e.g. a stretched spring) is captured at frame 0 / time 0. Call once
    // before the first Step(); subsequent Log(dt) calls record post-step state.
    public void LogInitial()
    {
        latest = Sampler();

        if (csv == null) return;

        if (!headerWritten) WriteHeader();
        WriteSampleRow();
    }

    public void Log(float dt)
    {
        latest = Sampler();
        frameNumber++;
        simTime += dt;

        if (csv == null) return;

        // Header is deferred until the first sample because the per-spring
        // columns depend on the spring count.
        if (!headerWritten) WriteHeader();

        WriteSampleRow();
    }

    private void WriteSampleRow()
    {
        var sb = new StringBuilder();
        sb.AppendFormat(Inv, "{0};{1:F6};{2};{3:F6};{4:F6};{5:F6};{6:F6}",
            frameNumber, simTime, latest.count,
            TotalCurrent(), TotalRest(), MaxStretchRatio(), AvgStretchRatio());
        if (writePerSpring)
            for (int i = 0; i < latest.count; i++)
                sb.AppendFormat(Inv, ";{0:F6}", latest.currentLengths[i]);
        csv.WriteLine(sb.ToString());
    }

    void OnGUI()
    {
        if (!showOverlay) return;
        if (style == null)
        {
            style = new GUIStyle(GUI.skin.label) { fontSize = 14 };
            style.normal.textColor = Color.white;
        }
        GUI.Label(new Rect(10, overlayY, 700, 20),
            string.Format(Inv,
                "{0} length  total={1:F3}  rest={2:F3}  maxStretch={3:F3}x  avgStretch={4:F3}x",
                label, TotalCurrent(), TotalRest(), MaxStretchRatio(), AvgStretchRatio()),
            style);
    }

    private float TotalCurrent()
    {
        float t = 0f;
        if (latest.currentLengths != null)
            for (int i = 0; i < latest.count; i++) t += latest.currentLengths[i];
        return t;
    }

    private float TotalRest()
    {
        float t = 0f;
        if (latest.restLengths != null)
            for (int i = 0; i < latest.count; i++) t += latest.restLengths[i];
        return t;
    }

    private float MaxStretchRatio()
    {
        float max = 0f;
        if (latest.currentLengths != null)
            for (int i = 0; i < latest.count; i++)
            {
                float rest = latest.restLengths[i];
                if (rest <= 0f) continue;
                float ratio = latest.currentLengths[i] / rest;
                if (ratio > max) max = ratio;
            }
        return max;
    }

    private float AvgStretchRatio()
    {
        float sum = 0f;
        int n = 0;
        if (latest.currentLengths != null)
            for (int i = 0; i < latest.count; i++)
            {
                float rest = latest.restLengths[i];
                if (rest <= 0f) continue;
                sum += latest.currentLengths[i] / rest;
                n++;
            }
        return n > 0 ? sum / n : 0f;
    }

    private void WriteHeader()
    {
        var sb = new StringBuilder();
        sb.Append("frame;time;springs;totalCurrent;totalRest;maxStretchRatio;avgStretchRatio"); // lengths in meters, time in seconds
        if (writePerSpring)
            for (int i = 0; i < latest.count; i++) sb.AppendFormat(Inv, ";len{0}", i);
        csv.WriteLine(sb.ToString());

        headerWritten = true;
    }

    private void OpenCsv()
    {
        var dir = Path.GetFullPath(Path.Combine(Application.dataPath, "../../Data-analysis/Logs/SpringLengthLogs"));
        Directory.CreateDirectory(dir);
        string safe = string.IsNullOrEmpty(label) ? "Sim" : label;
        foreach (char c in Path.GetInvalidFileNameChars()) safe = safe.Replace(c, '_');
        csvPath = Path.Combine(dir, $"{safe}_{DateTime.Now:yyyyMMdd_HHmmss}.csv");
        csv = new StreamWriter(csvPath, false);
        Debug.Log($"[SpringLengthLogger] {label}: writing spring-length log to {csvPath}");
    }

    private void CloseCsv()
    {
        if (csv == null) return;
        csv.Flush();
        csv.Close();
        csv = null;
    }
}
