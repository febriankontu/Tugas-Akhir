import 'dart:math';
import 'package:flutter/material.dart';
import 'package:cloud_firestore/cloud_firestore.dart';
import 'package:maps_toolkit/maps_toolkit.dart' as mp;

class SamplePoint {
  final double lat;
  final double lng;
  final double value;
  SamplePoint({required this.lat, required this.lng, required this.value});
}

class IDWPage extends StatefulWidget {
  const IDWPage({super.key});
  @override
  State<IDWPage> createState() => _IDWPageState();
}

class _IDWPageState extends State<IDWPage> {
  final FirebaseFirestore _firestore = FirebaseFirestore.instance;
  List<SamplePoint> _samples = [];
  List<double> _gridValues = [];
  bool _isLoading = false;
  int _gridRows = 25; // Resolusi Peta (Semakin tinggi semakin halus)
  int _gridCols = 25;

  @override
  void initState() {
    super.initState();
    _fetchData();
  }

  Future<void> _fetchData() async {
    setState(() => _isLoading = true);
    try {
      // Mengambil data riwayat
      var snapshot = await _firestore.collection('Data_testing').where('tipe', isEqualTo: 'riwayat').get();
      List<SamplePoint> temp = [];
      for (var doc in snapshot.docs) {
        var d = doc.data();
        if (d['latitude'] != null && d['nitrogen'] != null) {
          temp.add(SamplePoint(
            lat: (d['latitude'] as num).toDouble(),
            lng: (d['longitude'] as num).toDouble(),
            value: (d['nitrogen'] as num).toDouble(),
          ));
        }
      }
      setState(() {
        _samples = temp;
        _isLoading = false;
      });
    } catch (e) {
      print(e);
      setState(() => _isLoading = false);
    }
  }

  Future<void> _generateMap() async {
    if (_samples.isEmpty) return;
    setState(() => _isLoading = true);
    await Future.delayed(const Duration(milliseconds: 100)); // UI Breath

    // 1. Tentukan Kotak Peta
    double minLat = _samples.first.lat;
    double maxLat = _samples.first.lat;
    double minLng = _samples.first.lng;
    double maxLng = _samples.first.lng;

    for (var p in _samples) {
      if (p.lat < minLat) minLat = p.lat;
      if (p.lat > maxLat) maxLat = p.lat;
      if (p.lng < minLng) minLng = p.lng;
      if (p.lng > maxLng) maxLng = p.lng;
    }

    // Padding
    double pad = 0.00005;
    minLat -= pad; maxLat += pad; minLng -= pad; maxLng += pad;

    double latStep = (maxLat - minLat) / _gridRows;
    double lngStep = (maxLng - minLng) / _gridCols;
    List<double> results = [];

    // 2. Loop Grid & IDW
    for (int i = 0; i < _gridRows; i++) {
      for (int j = 0; j < _gridCols; j++) {
        double cLat = minLat + (i * latStep);
        double cLng = minLng + (j * lngStep);

        // Rumus IDW Sederhana
        double num = 0, den = 0;
        for (var s in _samples) {
          double dist = sqrt(pow(cLat - s.lat, 2) + pow(cLng - s.lng, 2));
          if (dist == 0) dist = 0.0000001;
          double w = 1 / (dist * dist); // Power = 2
          num += w * s.value;
          den += w;
        }
        results.add(num/den);
      }
    }

    setState(() {
      _gridValues = results;
      _isLoading = false;
    });
  }

  Color _getColor(double val) {
    // Skala Warna NPK
    if (val < 50) return Colors.red;
    if (val < 100) return Colors.orange;
    if (val < 150) return Colors.yellow;
    return Colors.green;
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text("2. Peta Sebaran NPK"), backgroundColor: Colors.indigo, foregroundColor: Colors.white),
      body: Column(
        children: [
          if (_samples.isNotEmpty)
            Container(
              padding: const EdgeInsets.all(10),
              color: Colors.indigo[50],
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  Text("Data Terkumpul: ${_samples.length} Titik"),
                  ElevatedButton(onPressed: _generateMap, child: const Text("GENERATE PETA"))
                ],
              ),
            ),
          Expanded(
            child: _isLoading
                ? const Center(child: CircularProgressIndicator())
                : _gridValues.isEmpty
                ? const Center(child: Text("Klik GENERATE PETA untuk melihat hasil."))
                : GridView.builder(
              physics: const NeverScrollableScrollPhysics(),
              gridDelegate: SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: _gridCols),
              itemCount: _gridValues.length,
              itemBuilder: (c, i) => Container(color: _getColor(_gridValues[i])),
            ),
          ),
          // Legenda
          Padding(
            padding: const EdgeInsets.all(8.0),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                _legend(Colors.red, "Kurang"),
                const SizedBox(width: 10),
                _legend(Colors.green, "Subur"),
              ],
            ),
          )
        ],
      ),
    );
  }
  Widget _legend(Color color, String text) {
    return Row(children: [Container(width: 15, height: 15, color: color), const SizedBox(width: 5), Text(text)]);
  }
}