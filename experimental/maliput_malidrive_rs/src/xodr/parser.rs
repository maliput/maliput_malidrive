//! XODR XML parser module.
//!
//! This module provides functionality to parse OpenDRIVE (.xodr) files
//! into the data structures defined in the xodr module.

use std::fs;
use std::io::Read;
use std::path::Path;

use quick_xml::events::{BytesStart, Event};
use quick_xml::Reader;

use nalgebra::Vector2;

use crate::common::{MalidriveError, MalidriveResult};
use crate::xodr::{
    Connection, ConnectionLaneLink, ContactPoint, Elevation, ElevationProfile, ElementType,
    Geometry, Header, Junction, JunctionType, Lane, LaneLink, LaneOffset,
    LaneSection, LaneType, LaneWidth, Lanes, LateralProfile, LinkElement, PRange, ParamPoly3,
    PlanView, RoadHeader, RoadLink, RoadType, RoadTypeType, Speed, Superelevation, Unit,
};

/// Parser configuration options.
#[derive(Debug, Clone)]
pub struct ParserConfiguration {
    /// Tolerance for geometric comparisons.
    pub tolerance: Option<f64>,
    /// Whether to allow schema errors.
    pub allow_schema_errors: bool,
    /// Whether to allow semantic errors.
    pub allow_semantic_errors: bool,
}

impl Default for ParserConfiguration {
    fn default() -> Self {
        Self {
            tolerance: Some(1e-3),
            allow_schema_errors: false,
            allow_semantic_errors: false,
        }
    }
}

impl ParserConfiguration {
    /// Creates a permissive configuration that allows errors.
    pub fn permissive() -> Self {
        Self {
            tolerance: Some(1e-3),
            allow_schema_errors: true,
            allow_semantic_errors: true,
        }
    }
}

/// Helper function to get an attribute value from XML element.
fn get_attribute(e: &BytesStart, name: &str) -> Option<String> {
    e.attributes()
        .filter_map(|a| a.ok())
        .find(|a| a.key.as_ref() == name.as_bytes())
        .map(|a| String::from_utf8_lossy(&a.value).to_string())
}

/// Helper function to get a required attribute or return error.
fn get_required_attribute(e: &BytesStart, name: &str, element: &str) -> MalidriveResult<String> {
    get_attribute(e, name).ok_or_else(|| {
        MalidriveError::ParsingError(format!(
            "Missing required attribute '{}' in element '{}'",
            name, element
        ))
    })
}

/// Helper function to parse an attribute as f64.
fn parse_f64_attribute(e: &BytesStart, name: &str, element: &str) -> MalidriveResult<f64> {
    let value = get_required_attribute(e, name, element)?;
    value.parse::<f64>().map_err(|_| {
        MalidriveError::ParsingError(format!(
            "Invalid float value '{}' for attribute '{}' in element '{}'",
            value, name, element
        ))
    })
}

/// Helper function to parse an optional attribute as f64.
fn parse_optional_f64_attribute(e: &BytesStart, name: &str) -> Option<f64> {
    get_attribute(e, name).and_then(|v| v.parse::<f64>().ok())
}

/// Helper function to parse an attribute as i32.
fn parse_i32_attribute(e: &BytesStart, name: &str, element: &str) -> MalidriveResult<i32> {
    let value = get_required_attribute(e, name, element)?;
    value.parse::<i32>().map_err(|_| {
        MalidriveError::ParsingError(format!(
            "Invalid integer value '{}' for attribute '{}' in element '{}'",
            value, name, element
        ))
    })
}

/// Helper function to parse an optional attribute as i32.
fn parse_optional_i32_attribute(e: &BytesStart, name: &str) -> Option<i32> {
    get_attribute(e, name).and_then(|v| v.parse::<i32>().ok())
}

/// XODR file parser.
pub struct XodrParser {
    #[allow(dead_code)]
    config: ParserConfiguration,
}

impl XodrParser {
    /// Creates a new parser with default configuration.
    pub fn new() -> Self {
        Self {
            config: ParserConfiguration::default(),
        }
    }

    /// Creates a new parser with the given configuration.
    pub fn with_config(config: ParserConfiguration) -> Self {
        Self { config }
    }

    /// Parses an XODR file from a path.
    pub fn parse_file<P: AsRef<Path>>(&self, path: P) -> MalidriveResult<XodrDocument> {
        let mut file = fs::File::open(path.as_ref()).map_err(|e| {
            MalidriveError::ParsingError(format!("Failed to open file: {}", e))
        })?;
        let mut contents = String::new();
        file.read_to_string(&mut contents).map_err(|e| {
            MalidriveError::ParsingError(format!("Failed to read file: {}", e))
        })?;
        self.parse_str(&contents)
    }

    /// Parses an XODR document from a string.
    pub fn parse_str(&self, xml: &str) -> MalidriveResult<XodrDocument> {
        let mut reader = Reader::from_str(xml);
        reader.trim_text(true);

        let mut document = XodrDocument::default();
        let mut buf = Vec::new();

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(ref e)) => {
                    match e.name().as_ref() {
                        b"header" => {
                            document.header = self.parse_header(e)?;
                        }
                        b"road" => {
                            let road = self.parse_road(&mut reader, e)?;
                            document.roads.push(road);
                        }
                        b"junction" => {
                            let junction = self.parse_junction(&mut reader, e)?;
                            document.junctions.push(junction);
                        }
                        _ => {}
                    }
                }
                Ok(Event::Empty(ref e)) => {
                    if e.name().as_ref() == b"header" {
                        document.header = self.parse_header(e)?;
                    }
                }
                Ok(Event::Eof) => break,
                Err(e) => {
                    return Err(MalidriveError::ParsingError(format!(
                        "XML parsing error: {}",
                        e
                    )));
                }
                _ => {}
            }
            buf.clear();
        }

        Ok(document)
    }

    /// Parses the header element.
    fn parse_header(&self, e: &BytesStart) -> MalidriveResult<Header> {
        let rev_major = parse_optional_i32_attribute(e, "revMajor").unwrap_or(1) as u32;
        let rev_minor = parse_optional_i32_attribute(e, "revMinor").unwrap_or(0) as u32;
        let name = get_attribute(e, "name");
        let version = get_attribute(e, "version");
        let date = get_attribute(e, "date");
        let north = parse_optional_f64_attribute(e, "north");
        let south = parse_optional_f64_attribute(e, "south");
        let east = parse_optional_f64_attribute(e, "east");
        let west = parse_optional_f64_attribute(e, "west");

        let mut header = Header::new(rev_major, rev_minor);
        header.name = name;
        header.version = version;
        header.date = date;
        header.north = north;
        header.south = south;
        header.east = east;
        header.west = west;

        Ok(header)
    }

    /// Parses a road element.
    fn parse_road(
        &self,
        reader: &mut Reader<&[u8]>,
        start: &BytesStart,
    ) -> MalidriveResult<RoadHeader> {
        let id = get_required_attribute(start, "id", "road")?;
        let name = get_attribute(start, "name");
        let length = parse_f64_attribute(start, "length", "road")?;
        let junction_id = get_attribute(start, "junction");

        let mut road = RoadHeader::new(id, length);
        if let Some(n) = name {
            road.name = Some(n);
        }
        if let Some(jid) = junction_id {
            if jid != "-1" {
                road.junction = jid;
            }
        }

        let mut buf = Vec::new();
        let mut depth = 1;

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(ref e)) => {
                    depth += 1;
                    match e.name().as_ref() {
                        b"link" => {
                            let link = self.parse_road_link(reader)?;
                            road.link = Some(link);
                            depth -= 1; // parse_road_link consumes the end tag
                        }
                        b"type" => {
                            let road_type = self.parse_road_type(reader, e)?;
                            road.road_types.push(road_type);
                            depth -= 1;
                        }
                        b"planView" => {
                            let plan_view = self.parse_plan_view(reader)?;
                            road.plan_view = plan_view;
                            depth -= 1;
                        }
                        b"elevationProfile" => {
                            let profile = self.parse_elevation_profile(reader)?;
                            road.elevation_profile = profile;
                            depth -= 1;
                        }
                        b"lateralProfile" => {
                            let profile = self.parse_lateral_profile(reader)?;
                            road.lateral_profile = profile;
                            depth -= 1;
                        }
                        b"lanes" => {
                            let lanes = self.parse_lanes(reader)?;
                            road.lanes = lanes;
                            depth -= 1;
                        }
                        _ => {}
                    }
                }
                Ok(Event::End(_)) => {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                Ok(Event::Eof) => break,
                Err(e) => {
                    return Err(MalidriveError::ParsingError(format!(
                        "XML parsing error in road: {}",
                        e
                    )));
                }
                _ => {}
            }
            buf.clear();
        }

        Ok(road)
    }

    /// Parses a road link element.
    fn parse_road_link(&self, reader: &mut Reader<&[u8]>) -> MalidriveResult<RoadLink> {
        let mut link = RoadLink::default();
        let mut buf = Vec::new();
        let mut depth = 1;

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                    match e.name().as_ref() {
                        b"predecessor" => {
                            let element_id = get_required_attribute(e, "elementId", "predecessor")?;
                            let element_type_str = get_attribute(e, "elementType")
                                .unwrap_or_else(|| "road".to_string());
                            let element_type = element_type_str
                                .parse::<ElementType>()
                                .unwrap_or(ElementType::Road);
                            let contact_point = get_attribute(e, "contactPoint")
                                .and_then(|s| s.parse::<ContactPoint>().ok());

                            link.predecessor = Some(LinkElement {
                                element_type,
                                element_id,
                                contact_point,
                            });
                        }
                        b"successor" => {
                            let element_id = get_required_attribute(e, "elementId", "successor")?;
                            let element_type_str = get_attribute(e, "elementType")
                                .unwrap_or_else(|| "road".to_string());
                            let element_type = element_type_str
                                .parse::<ElementType>()
                                .unwrap_or(ElementType::Road);
                            let contact_point = get_attribute(e, "contactPoint")
                                .and_then(|s| s.parse::<ContactPoint>().ok());

                            link.successor = Some(LinkElement {
                                element_type,
                                element_id,
                                contact_point,
                            });
                        }
                        _ => {}
                    }
                }
                Ok(Event::End(_)) => {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                Ok(Event::Eof) => break,
                Err(e) => {
                    return Err(MalidriveError::ParsingError(format!(
                        "XML parsing error in link: {}",
                        e
                    )));
                }
                _ => {}
            }
            buf.clear();
        }

        Ok(link)
    }

    /// Parses a road type element.
    fn parse_road_type(
        &self,
        reader: &mut Reader<&[u8]>,
        start: &BytesStart,
    ) -> MalidriveResult<RoadType> {
        let s = parse_f64_attribute(start, "s", "type")?;
        let type_str = get_attribute(start, "type").unwrap_or_else(|| "unknown".to_string());
        let road_type_type = type_str.parse::<RoadTypeType>().unwrap_or(RoadTypeType::Unknown);

        let mut road_type = RoadType::new(s, road_type_type);
        let mut buf = Vec::new();
        let mut depth = 1;

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                    if e.name().as_ref() == b"speed" {
                        let max = parse_f64_attribute(e, "max", "speed")?;
                        let unit_str = get_attribute(e, "unit").unwrap_or_else(|| "m/s".to_string());
                        let unit = unit_str.parse::<Unit>().unwrap_or(Unit::Ms);
                        road_type.speed = Some(Speed::new(max, unit));
                    }
                }
                Ok(Event::End(_)) => {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                Ok(Event::Eof) => break,
                Err(e) => {
                    return Err(MalidriveError::ParsingError(format!(
                        "XML parsing error in type: {}",
                        e
                    )));
                }
                _ => {}
            }
            buf.clear();
        }

        Ok(road_type)
    }

    /// Parses the planView element.
    fn parse_plan_view(&self, reader: &mut Reader<&[u8]>) -> MalidriveResult<PlanView> {
        let mut geometries = Vec::new();
        let mut buf = Vec::new();
        let mut depth = 1;

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(ref e)) => {
                    depth += 1;
                    if e.name().as_ref() == b"geometry" {
                        let geometry = self.parse_geometry(reader, e)?;
                        geometries.push(geometry);
                        depth -= 1; // parse_geometry consumes the end tag
                    }
                }
                Ok(Event::End(_)) => {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                Ok(Event::Eof) => break,
                Err(e) => {
                    return Err(MalidriveError::ParsingError(format!(
                        "XML parsing error in planView: {}",
                        e
                    )));
                }
                _ => {}
            }
            buf.clear();
        }

        Ok(PlanView { geometries })
    }

    /// Parses a geometry element.
    fn parse_geometry(
        &self,
        reader: &mut Reader<&[u8]>,
        start: &BytesStart,
    ) -> MalidriveResult<Geometry> {
        let s = parse_f64_attribute(start, "s", "geometry")?;
        let x = parse_f64_attribute(start, "x", "geometry")?;
        let y = parse_f64_attribute(start, "y", "geometry")?;
        let hdg = parse_f64_attribute(start, "hdg", "geometry")?;
        let length = parse_f64_attribute(start, "length", "geometry")?;

        let start_point = Vector2::new(x, y);
        let mut geometry: Option<Geometry> = None;
        let mut buf = Vec::new();
        let mut depth = 1;

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                    match e.name().as_ref() {
                        b"line" => {
                            geometry = Some(Geometry::new_line(s, start_point, hdg, length));
                        }
                        b"arc" => {
                            let curvature = parse_f64_attribute(e, "curvature", "arc")?;
                            geometry =
                                Some(Geometry::new_arc(s, start_point, hdg, length, curvature));
                        }
                        b"spiral" => {
                            let curv_start = parse_f64_attribute(e, "curvStart", "spiral")?;
                            let curv_end = parse_f64_attribute(e, "curvEnd", "spiral")?;
                            geometry = Some(Geometry::new_spiral(
                                s, start_point, hdg, length, curv_start, curv_end,
                            ));
                        }
                        b"paramPoly3" => {
                            let a_u = parse_f64_attribute(e, "aU", "paramPoly3")?;
                            let b_u = parse_f64_attribute(e, "bU", "paramPoly3")?;
                            let c_u = parse_f64_attribute(e, "cU", "paramPoly3")?;
                            let d_u = parse_f64_attribute(e, "dU", "paramPoly3")?;
                            let a_v = parse_f64_attribute(e, "aV", "paramPoly3")?;
                            let b_v = parse_f64_attribute(e, "bV", "paramPoly3")?;
                            let c_v = parse_f64_attribute(e, "cV", "paramPoly3")?;
                            let d_v = parse_f64_attribute(e, "dV", "paramPoly3")?;
                            let p_range = get_attribute(e, "pRange")
                                .and_then(|s| s.parse::<PRange>().ok())
                                .unwrap_or(PRange::Normalized);
                            let param_poly3 = ParamPoly3::new(
                                a_u, b_u, c_u, d_u, a_v, b_v, c_v, d_v, p_range,
                            );
                            geometry = Some(Geometry::new_param_poly3(
                                s, start_point, hdg, length, param_poly3,
                            ));
                        }
                        _ => {}
                    }
                }
                Ok(Event::End(_)) => {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                Ok(Event::Eof) => break,
                Err(e) => {
                    return Err(MalidriveError::ParsingError(format!(
                        "XML parsing error in geometry: {}",
                        e
                    )));
                }
                _ => {}
            }
            buf.clear();
        }

        // Default to line if no geometry type was specified
        Ok(geometry.unwrap_or_else(|| Geometry::new_line(s, start_point, hdg, length)))
    }

    /// Parses the elevationProfile element.
    fn parse_elevation_profile(
        &self,
        reader: &mut Reader<&[u8]>,
    ) -> MalidriveResult<ElevationProfile> {
        let mut elevations = Vec::new();
        let mut buf = Vec::new();
        let mut depth = 1;

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                    if e.name().as_ref() == b"elevation" {
                        let s = parse_f64_attribute(e, "s", "elevation")?;
                        let a = parse_f64_attribute(e, "a", "elevation")?;
                        let b = parse_f64_attribute(e, "b", "elevation")?;
                        let c = parse_f64_attribute(e, "c", "elevation")?;
                        let d = parse_f64_attribute(e, "d", "elevation")?;
                        elevations.push(Elevation::new(s, a, b, c, d));
                    }
                }
                Ok(Event::End(_)) => {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                Ok(Event::Eof) => break,
                Err(e) => {
                    return Err(MalidriveError::ParsingError(format!(
                        "XML parsing error in elevationProfile: {}",
                        e
                    )));
                }
                _ => {}
            }
            buf.clear();
        }

        Ok(ElevationProfile::new(elevations))
    }

    /// Parses the lateralProfile element.
    fn parse_lateral_profile(&self, reader: &mut Reader<&[u8]>) -> MalidriveResult<LateralProfile> {
        let mut superelevations = Vec::new();
        let mut buf = Vec::new();
        let mut depth = 1;

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                    if e.name().as_ref() == b"superelevation" {
                        let s = parse_f64_attribute(e, "s", "superelevation")?;
                        let a = parse_f64_attribute(e, "a", "superelevation")?;
                        let b = parse_f64_attribute(e, "b", "superelevation")?;
                        let c = parse_f64_attribute(e, "c", "superelevation")?;
                        let d = parse_f64_attribute(e, "d", "superelevation")?;
                        superelevations.push(Superelevation::new(s, a, b, c, d));
                    }
                }
                Ok(Event::End(_)) => {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                Ok(Event::Eof) => break,
                Err(e) => {
                    return Err(MalidriveError::ParsingError(format!(
                        "XML parsing error in lateralProfile: {}",
                        e
                    )));
                }
                _ => {}
            }
            buf.clear();
        }

        Ok(LateralProfile::new(superelevations))
    }

    /// Parses the lanes element.
    fn parse_lanes(&self, reader: &mut Reader<&[u8]>) -> MalidriveResult<Lanes> {
        let mut lane_offsets = Vec::new();
        let mut lane_sections = Vec::new();
        let mut buf = Vec::new();
        let mut depth = 1;

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(ref e)) => {
                    depth += 1;
                    match e.name().as_ref() {
                        b"laneOffset" => {
                            let s = parse_f64_attribute(e, "s", "laneOffset")?;
                            let a = parse_f64_attribute(e, "a", "laneOffset")?;
                            let b = parse_f64_attribute(e, "b", "laneOffset")?;
                            let c = parse_f64_attribute(e, "c", "laneOffset")?;
                            let d = parse_f64_attribute(e, "d", "laneOffset")?;
                            lane_offsets.push(LaneOffset::new(s, a, b, c, d));
                            depth -= 1; // Empty element typically
                        }
                        b"laneSection" => {
                            let section = self.parse_lane_section(reader, e)?;
                            lane_sections.push(section);
                            depth -= 1;
                        }
                        _ => {}
                    }
                }
                Ok(Event::Empty(ref e)) => {
                    if e.name().as_ref() == b"laneOffset" {
                        let s = parse_f64_attribute(e, "s", "laneOffset")?;
                        let a = parse_f64_attribute(e, "a", "laneOffset")?;
                        let b = parse_f64_attribute(e, "b", "laneOffset")?;
                        let c = parse_f64_attribute(e, "c", "laneOffset")?;
                        let d = parse_f64_attribute(e, "d", "laneOffset")?;
                        lane_offsets.push(LaneOffset::new(s, a, b, c, d));
                    }
                }
                Ok(Event::End(_)) => {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                Ok(Event::Eof) => break,
                Err(e) => {
                    return Err(MalidriveError::ParsingError(format!(
                        "XML parsing error in lanes: {}",
                        e
                    )));
                }
                _ => {}
            }
            buf.clear();
        }

        Ok(Lanes {
            lane_offsets,
            lane_sections,
        })
    }

    /// Parses a laneSection element.
    fn parse_lane_section(
        &self,
        reader: &mut Reader<&[u8]>,
        start: &BytesStart,
    ) -> MalidriveResult<LaneSection> {
        let s = parse_f64_attribute(start, "s", "laneSection")?;
        let mut section = LaneSection::new(s);
        let mut buf = Vec::new();
        let mut depth = 1;

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(ref e)) => {
                    depth += 1;
                    match e.name().as_ref() {
                        b"left" => {
                            let lanes = self.parse_lane_group(reader)?;
                            for lane in lanes {
                                let _ = section.add_lane(lane);
                            }
                            depth -= 1;
                        }
                        b"center" => {
                            let lanes = self.parse_lane_group(reader)?;
                            for lane in lanes {
                                let _ = section.add_lane(lane);
                            }
                            depth -= 1;
                        }
                        b"right" => {
                            let lanes = self.parse_lane_group(reader)?;
                            for lane in lanes {
                                let _ = section.add_lane(lane);
                            }
                            depth -= 1;
                        }
                        _ => {}
                    }
                }
                Ok(Event::End(_)) => {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                Ok(Event::Eof) => break,
                Err(e) => {
                    return Err(MalidriveError::ParsingError(format!(
                        "XML parsing error in laneSection: {}",
                        e
                    )));
                }
                _ => {}
            }
            buf.clear();
        }

        Ok(section)
    }

    /// Parses a lane group (left, center, or right).
    fn parse_lane_group(&self, reader: &mut Reader<&[u8]>) -> MalidriveResult<Vec<Lane>> {
        let mut lanes = Vec::new();
        let mut buf = Vec::new();
        let mut depth = 1;

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(ref e)) => {
                    depth += 1;
                    if e.name().as_ref() == b"lane" {
                        let lane = self.parse_lane(reader, e)?;
                        lanes.push(lane);
                        depth -= 1;
                    }
                }
                Ok(Event::End(_)) => {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                Ok(Event::Eof) => break,
                Err(e) => {
                    return Err(MalidriveError::ParsingError(format!(
                        "XML parsing error in lane group: {}",
                        e
                    )));
                }
                _ => {}
            }
            buf.clear();
        }

        Ok(lanes)
    }

    /// Parses a lane element.
    fn parse_lane(&self, reader: &mut Reader<&[u8]>, start: &BytesStart) -> MalidriveResult<Lane> {
        let id = parse_i32_attribute(start, "id", "lane")?;
        let type_str = get_attribute(start, "type").unwrap_or_else(|| "none".to_string());
        let lane_type = type_str.parse::<LaneType>().unwrap_or(LaneType::None);

        let mut lane = Lane::new(id, lane_type);
        let mut buf = Vec::new();
        let mut depth = 1;

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(ref e)) => {
                    depth += 1;
                    match e.name().as_ref() {
                        b"link" => {
                            let link = self.parse_lane_link(reader)?;
                            lane.link = Some(link);
                            depth -= 1;
                        }
                        b"width" => {
                            // width is usually empty, but could have content
                            let width = self.parse_lane_width(e)?;
                            lane.widths.push(width);
                            depth -= 1; // Assuming no children
                        }
                        _ => {}
                    }
                }
                Ok(Event::Empty(ref e)) => {
                    if e.name().as_ref() == b"width" {
                        let width = self.parse_lane_width(e)?;
                        lane.widths.push(width);
                    }
                }
                Ok(Event::End(_)) => {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                Ok(Event::Eof) => break,
                Err(e) => {
                    return Err(MalidriveError::ParsingError(format!(
                        "XML parsing error in lane: {}",
                        e
                    )));
                }
                _ => {}
            }
            buf.clear();
        }

        Ok(lane)
    }

    /// Parses a lane width element.
    fn parse_lane_width(&self, e: &BytesStart) -> MalidriveResult<LaneWidth> {
        let s_offset = parse_f64_attribute(e, "sOffset", "width")?;
        let a = parse_f64_attribute(e, "a", "width")?;
        let b = parse_f64_attribute(e, "b", "width")?;
        let c = parse_f64_attribute(e, "c", "width")?;
        let d = parse_f64_attribute(e, "d", "width")?;
        Ok(LaneWidth::new(s_offset, a, b, c, d))
    }

    /// Parses a lane link element.
    fn parse_lane_link(&self, reader: &mut Reader<&[u8]>) -> MalidriveResult<LaneLink> {
        let mut predecessor_id = None;
        let mut successor_id = None;
        let mut buf = Vec::new();
        let mut depth = 1;

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                    match e.name().as_ref() {
                        b"predecessor" => {
                            if let Some(id) = parse_optional_i32_attribute(e, "id") {
                                predecessor_id = Some(id);
                            }
                        }
                        b"successor" => {
                            if let Some(id) = parse_optional_i32_attribute(e, "id") {
                                successor_id = Some(id);
                            }
                        }
                        _ => {}
                    }
                }
                Ok(Event::End(_)) => {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                Ok(Event::Eof) => break,
                Err(e) => {
                    return Err(MalidriveError::ParsingError(format!(
                        "XML parsing error in lane link: {}",
                        e
                    )));
                }
                _ => {}
            }
            buf.clear();
        }

        Ok(LaneLink::new(predecessor_id, successor_id))
    }

    /// Parses a junction element.
    fn parse_junction(
        &self,
        reader: &mut Reader<&[u8]>,
        start: &BytesStart,
    ) -> MalidriveResult<Junction> {
        let id = get_required_attribute(start, "id", "junction")?;
        let name = get_attribute(start, "name");
        let junction_type_str = get_attribute(start, "type");
        let junction_type = junction_type_str
            .and_then(|s| s.parse::<JunctionType>().ok())
            .unwrap_or(JunctionType::Default);

        let mut junction = Junction::new(id);
        junction.name = name;
        junction.junction_type = junction_type;

        let mut buf = Vec::new();
        let mut depth = 1;

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(ref e)) => {
                    depth += 1;
                    if e.name().as_ref() == b"connection" {
                        let connection = self.parse_connection(reader, e)?;
                        junction.connections.push(connection);
                        depth -= 1;
                    }
                }
                Ok(Event::End(_)) => {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                Ok(Event::Eof) => break,
                Err(e) => {
                    return Err(MalidriveError::ParsingError(format!(
                        "XML parsing error in junction: {}",
                        e
                    )));
                }
                _ => {}
            }
            buf.clear();
        }

        Ok(junction)
    }

    /// Parses a connection element.
    fn parse_connection(
        &self,
        reader: &mut Reader<&[u8]>,
        start: &BytesStart,
    ) -> MalidriveResult<Connection> {
        let id = get_required_attribute(start, "id", "connection")?;
        let incoming_road = get_required_attribute(start, "incomingRoad", "connection")?;
        let connecting_road = get_required_attribute(start, "connectingRoad", "connection")?;
        let contact_point_str = get_attribute(start, "contactPoint");
        let contact_point = contact_point_str
            .and_then(|s| s.parse::<ContactPoint>().ok())
            .unwrap_or(ContactPoint::Start);

        let mut connection = Connection::new(id, incoming_road, connecting_road, contact_point);

        let mut buf = Vec::new();
        let mut depth = 1;

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                    if e.name().as_ref() == b"laneLink" {
                        let from = parse_i32_attribute(e, "from", "laneLink")?;
                        let to = parse_i32_attribute(e, "to", "laneLink")?;
                        connection.lane_links.push(ConnectionLaneLink::new(from, to));
                    }
                }
                Ok(Event::End(_)) => {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                Ok(Event::Eof) => break,
                Err(e) => {
                    return Err(MalidriveError::ParsingError(format!(
                        "XML parsing error in connection: {}",
                        e
                    )));
                }
                _ => {}
            }
            buf.clear();
        }

        Ok(connection)
    }
}

impl Default for XodrParser {
    fn default() -> Self {
        Self::new()
    }
}

/// Represents a parsed XODR document.
#[derive(Debug, Clone, Default)]
pub struct XodrDocument {
    /// The document header.
    pub header: Header,
    /// The roads in the document.
    pub roads: Vec<RoadHeader>,
    /// The junctions in the document.
    pub junctions: Vec<Junction>,
}

impl XodrDocument {
    /// Returns the road with the given ID, if it exists.
    pub fn get_road(&self, id: &str) -> Option<&RoadHeader> {
        self.roads.iter().find(|r| r.id == id)
    }

    /// Returns the junction with the given ID, if it exists.
    pub fn get_junction(&self, id: &str) -> Option<&Junction> {
        self.junctions.iter().find(|j| j.id == id)
    }

    /// Returns all drivable lanes across all roads.
    pub fn get_all_drivable_lanes(&self) -> Vec<(&RoadHeader, &LaneSection, &Lane)> {
        let mut result = Vec::new();
        for road in &self.roads {
            for section in &road.lanes.lane_sections {
                for lane in section.all_lanes() {
                    if lane.lane_type.is_drivable() {
                        result.push((road, section, lane));
                    }
                }
            }
        }
        result
    }

    /// Returns the total length of all roads.
    pub fn total_road_length(&self) -> f64 {
        self.roads.iter().map(|r| r.length).sum()
    }
}

/// Loads an XODR document from a file path.
pub fn load_from_file<P: AsRef<Path>>(path: P) -> MalidriveResult<XodrDocument> {
    XodrParser::new().parse_file(path)
}

/// Loads an XODR document from a string.
pub fn load_from_str(xml: &str) -> MalidriveResult<XodrDocument> {
    XodrParser::new().parse_str(xml)
}

#[cfg(test)]
mod tests {
    use super::*;

    const SIMPLE_XODR: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="1" name="TestMap" version="1.00"/>
    <road name="TestRoad" length="100.0" id="1" junction="-1">
        <link/>
        <planView>
            <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0">
                <line/>
            </geometry>
        </planView>
        <elevationProfile/>
        <lateralProfile/>
        <lanes>
            <laneSection s="0.0">
                <left>
                    <lane id="1" type="driving" level="0">
                        <link/>
                        <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
                    </lane>
                </left>
                <center>
                    <lane id="0" type="none" level="0">
                        <link/>
                    </lane>
                </center>
                <right>
                    <lane id="-1" type="driving" level="0">
                        <link/>
                        <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
                    </lane>
                </right>
            </laneSection>
        </lanes>
    </road>
</OpenDRIVE>"#;

    #[test]
    fn test_parse_simple_xodr() {
        let doc = load_from_str(SIMPLE_XODR).unwrap();

        assert_eq!(doc.header.rev_major, 1);
        assert_eq!(doc.header.rev_minor, 1);
        assert_eq!(doc.header.name.as_deref(), Some("TestMap"));

        assert_eq!(doc.roads.len(), 1);
        let road = &doc.roads[0];
        assert_eq!(road.id, "1");
        assert_eq!(road.name.as_deref(), Some("TestRoad"));
        assert_eq!(road.length, 100.0);

        assert_eq!(road.plan_view.geometries.len(), 1);
        let geom = &road.plan_view.geometries[0];
        assert_eq!(geom.s_0, 0.0);
        assert_eq!(geom.start_point.x, 0.0);
        assert_eq!(geom.start_point.y, 0.0);

        assert_eq!(road.lanes.lane_sections.len(), 1);
        let section = &road.lanes.lane_sections[0];
        assert_eq!(section.left_lanes.len(), 1);
        assert_eq!(section.right_lanes.len(), 1);
        assert_eq!(section.center_lane.id, 0); // Center lane exists with id 0
    }

    #[test]
    fn test_parse_arc_geometry() {
        let xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="1"/>
    <road name="ArcRoad" length="50.0" id="2" junction="-1">
        <planView>
            <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="50.0">
                <arc curvature="0.01"/>
            </geometry>
        </planView>
        <lanes>
            <laneSection s="0.0">
                <center>
                    <lane id="0" type="none"/>
                </center>
            </laneSection>
        </lanes>
    </road>
</OpenDRIVE>"#;

        let doc = load_from_str(xml).unwrap();
        let road = &doc.roads[0];
        let geom = &road.plan_view.geometries[0];

        assert!(geom.is_arc());
        if let Some(arc) = geom.as_arc() {
            assert!((arc.curvature - 0.01).abs() < 1e-9);
        }
    }

    #[test]
    fn test_parse_spiral_geometry() {
        let xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="1"/>
    <road name="SpiralRoad" length="30.0" id="3" junction="-1">
        <planView>
            <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="30.0">
                <spiral curvStart="0.0" curvEnd="0.02"/>
            </geometry>
        </planView>
        <lanes>
            <laneSection s="0.0">
                <center>
                    <lane id="0" type="none"/>
                </center>
            </laneSection>
        </lanes>
    </road>
</OpenDRIVE>"#;

        let doc = load_from_str(xml).unwrap();
        let road = &doc.roads[0];
        let geom = &road.plan_view.geometries[0];

        assert!(geom.is_spiral());
        if let Some(spiral) = geom.as_spiral() {
            assert!((spiral.curv_start - 0.0).abs() < 1e-9);
            assert!((spiral.curv_end - 0.02).abs() < 1e-9);
        }
    }

    #[test]
    fn test_parse_elevation_profile() {
        let xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="1"/>
    <road name="ElevatedRoad" length="100.0" id="1" junction="-1">
        <planView>
            <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0">
                <line/>
            </geometry>
        </planView>
        <elevationProfile>
            <elevation s="0.0" a="0.0" b="0.1" c="0.0" d="0.0"/>
        </elevationProfile>
        <lanes>
            <laneSection s="0.0">
                <center>
                    <lane id="0" type="none"/>
                </center>
            </laneSection>
        </lanes>
    </road>
</OpenDRIVE>"#;

        let doc = load_from_str(xml).unwrap();
        let road = &doc.roads[0];

        assert_eq!(road.elevation_profile.elevations.len(), 1);
        let elev = &road.elevation_profile.elevations[0];
        assert_eq!(elev.s, 0.0);
        assert_eq!(elev.a, 0.0);
        assert!((elev.b - 0.1).abs() < 1e-9);
    }

    #[test]
    fn test_parse_junction() {
        let xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="1"/>
    <junction name="TestJunction" id="1" type="default">
        <connection id="1" incomingRoad="1" connectingRoad="2" contactPoint="start">
            <laneLink from="-1" to="-1"/>
        </connection>
    </junction>
</OpenDRIVE>"#;

        let doc = load_from_str(xml).unwrap();
        assert_eq!(doc.junctions.len(), 1);

        let junction = &doc.junctions[0];
        assert_eq!(junction.id, "1");
        assert_eq!(junction.name.as_deref(), Some("TestJunction"));
        assert!(matches!(junction.junction_type, JunctionType::Default));

        assert_eq!(junction.connections.len(), 1);
        let conn = &junction.connections[0];
        assert_eq!(conn.id, "1");
        assert_eq!(conn.incoming_road, "1");
        assert_eq!(conn.connecting_road, "2");
        assert!(matches!(conn.contact_point, ContactPoint::Start));

        assert_eq!(conn.lane_links.len(), 1);
        assert_eq!(conn.lane_links[0].from, -1);
        assert_eq!(conn.lane_links[0].to, -1);
    }

    #[test]
    fn test_get_all_drivable_lanes() {
        let doc = load_from_str(SIMPLE_XODR).unwrap();
        let drivable = doc.get_all_drivable_lanes();

        // Should have 2 driving lanes (left id=1 and right id=-1)
        assert_eq!(drivable.len(), 2);
    }

    #[test]
    fn test_total_road_length() {
        let doc = load_from_str(SIMPLE_XODR).unwrap();
        assert!((doc.total_road_length() - 100.0).abs() < 1e-9);
    }
}
