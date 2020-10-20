#pragma once

#include "gdkmm/general.h"
#include <gtkmm/drawingarea.h>
#include <sigc++/sigc++.h>
#include <vector>

///
///class to represent a hierarchy of objects drawable to a cairo canvas
///child drawables can be added and drawn
class Drawable : public sigc::trackable{
protected:
  /**
   * Children of this drawable object.  How these are managed and drawn
   * or even if they are drawn at all is implementation dependent.
   */
  std::vector<std::shared_ptr<Drawable>> children;

  /**
  *  Draws all of this drawables current children;
  */
  bool drawChildren(const Cairo::RefPtr<Cairo::Context>& cr) {
    bool out = false;
    for(auto& d: children){
      out = out || d->draw(cr);
    }
    return out;
  }

public:
  Drawable() = default;

  /**
   * The draw function.  Override to add functionality.
   */
  virtual bool draw(const Cairo::RefPtr<Cairo::Context>& cr) = 0;

  /**
   *  Connects this object to a drawing area.  After this is done, draw
   *  will be called on this object when the drawing area is redrawn
   */
  void connect(Gtk::DrawingArea& area){
    area.signal_draw().connect(sigc::mem_fun(*this, &Drawable::draw));
  }

  void addChild(std::shared_ptr<Drawable>&& d) {
    children.push_back(d);
  }

  void removeChild(int i) {
    children.erase(children.begin() + i);
  }

  const std::vector<std::shared_ptr<Drawable>>& getChildren(){
    return children;
  }
};

/**
 *  A drawable that just draws all of its children.  It may be useful to add this to a
 *  a drawing area then use it as an interface to manage what is and isn't drawn.
 */
class ChildDrawer: public Drawable{
public:
  ChildDrawer() = default;
  bool draw(const Cairo::RefPtr<Cairo::Context>& cr) override{
    return drawChildren(cr);
  }
};


///
/// An abstract drawable that has a position, orientation, scale, foreground
/// color, and background color
class Shape : public Drawable{
private:
  bool between(double x,double min, double max){
    return x>min && x<max;
  }

public:
  double x=0,y=0; ///< coordinates of center
  double angle=0; ///< the orientation of the object
  double fr = 0, fg = 0, fb = 0, fa = 1; ///< fill rgba
  double sr = 0, sg = 0, sb = 0, sa = 1; ///< stroke rgba
  double sx=1,sy=1; ///<size of the bounding box

  Shape() = default;

  Shape(double x, double y, double sx, double sy): x(x), y(y), sx(sx), sy(sy){}

  void set_position(double x, double y){
    this->x = x;
    this->y = y;
  }

  void set_fill_color(double r,double g, double b, double a){
    this->fr = r;
    this->fg = g;
    this->fb = b;
    this->fa = a;
  }

  void set_stroke_color(double r, double g, double b, double a) {
    this->sr = r;
    this->sg = g;
    this->sb = b;
    this->sa = a;
  }

  void set_dimensions(double sx, double sy){
    this->sx =sx; this->sy = sy;
  }

  void set_angle(double angle){
    this->angle = angle;
  }

  double get_angle(double angle){
    return angle;
  }

  /**
   *  Called after setting the cairo context to the color and transformation
   *  settings stored in this object. Override with shape specific draw code.
   */
  virtual bool draw_shape(const Cairo::RefPtr<Cairo::Context> &cr) = 0;

  virtual bool draw(const Cairo::RefPtr<Cairo::Context> &cr) override{
    cr->save();
    cr->translate(x, y);
    cr->scale(sx, sy);
    cr->rotate(angle);
    cr->set_source_rgb(fr, fg, fb);
    bool out = draw_shape(cr);
    cr->restore();
    return out;
  }

  double top(){
    return y - sy/2;
  }

  double bottom(){
    return y + sy / 2;
  }

  double left(){
    return x - sx /2;
  }

  double right(){
    return y + sx/2;
  }

  /**
   * Tests if a point is in the bounding box with the same center
   * as this object and width and height sx and sy
   */
  bool point_in_box(double x, double y) {
    return between(x, this->x - sx/2, this->x + sx/2) &&
           between(y, this->y - sy/2, this->y + sy/2);
  }

  /// by default tests the same as point_in_box but can be overridden
  /// to add more precise functionality
  virtual bool point_inside(double x, double y){
    return point_in_box(x,y);
  }
};

/**
 *  Class for a drawable square
 */
class Square : public Shape{
public:
  Square() = default;

  Square(double x, double y, double sx, double sy) : Shape(x, y, sx, sy) {  }

  virtual bool draw_shape(const Cairo::RefPtr<Cairo::Context> &cr) override{

    cr->rectangle(-0.5, -0.5, 1, 1);
    cr->close_path();
    cr->fill();
    cr->set_source_rgb(sr, sg, sb);
    //cr->stroke();
    return false;
  }
};

/**
 *  Class for a hexagon square
 */
class Hexagon : public Shape {
public:
  Hexagon() = default;

  Hexagon(double x, double y, double sx, double sy) : Shape(x, y, sx, sy) {}

  virtual bool draw_shape(const Cairo::RefPtr<Cairo::Context> &cr) override {
    cr->translate(-0.5, -0.5);
    cr->move_to(0.5 , 0);
    cr->line_to(1,1/(2*sqrt(3)));
    cr->line_to(1,1 - 1 / (2 * sqrt(3)));
    cr->line_to(0.5, 1);
    cr->line_to(0,1 - 1 / (2 * sqrt(3)));
    cr->line_to(0, 1 / (2 * sqrt(3)));
    cr->close_path();
    cr->fill();
    cr->set_source_rgb(sr, sg, sb);
    cr->stroke();
    return false;
  }
};

/**
 *  Class for a drawable picture.  Loads image data from a file.  Is not affected
 *  by setting the fill or stroke color.
 */
class Picture : public Shape{

  Glib::RefPtr<Gdk::Pixbuf> image;

public:
  Picture() = default;

  /**
   * Calls from_file and may trigger file exception
   */
  Picture(const std::string &path){
    from_file(path);
  }

  /**
   *  Draws the picture
   */
  bool draw_shape(const Cairo::RefPtr<Cairo::Context> &cr) override {
    cr->scale(1.0/image->get_width(),1.0/image->get_height());
    Gdk::Cairo::set_source_pixbuf(cr, image,-image->get_width()/2.0,-image->get_height()/2.0);
    cr->paint();
    return false;
  }

  /**
   *  Loads the image from the file at the passed path and may trigger
   *  a file exception if loading fails.
   */
  void from_file(const std::string &path){
    image = Gdk::Pixbuf::create_from_file(path);
  }
};


/// a wrapper for a drawing area that add a convienience function
/// for triggering a draw
class AreaController{
protected:
  Gtk::DrawingArea* area;
public:
  AreaController(Gtk::DrawingArea *area): area{area} {}

  /**
   *  Should trigger a draw in the underlying drawing area.
   */
  void invalidate_rect() {
    auto win = area->get_window();
    if(win)
      win->invalidate_rect(area->get_allocation(), true);
  }
};
